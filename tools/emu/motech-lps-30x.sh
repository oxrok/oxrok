# A Motech LPS-301 (emu/src/models/motech-lps-30x.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
# C registers one driver from this directory, motech-lps-301: its scan has the
# LPS-301 fixed, so no MODEL answer changes the tables and only that model's
# paths run.
EMU_MODEL=motech-lps-30x
EMU_CDC=1
EMU_DRIVER=motech-lps-301
EMU_GCOV=motech-lps-30x
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    LONG=0123456789012345678901234567890123456789012345678901234567890

    # A scan: MODEL, VERSION, STATUS, and no setting, which setters=0 would flag.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=2400/8n1 --scan
    emu_case "no such node" --keep-header --no-output -- -d motech-lps-301:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d motech-lps-301 --scan
    # sigrok-cli opens the port before it reads the parameters.
    emu_case "serialcomm no such thing" --keep-header --no-output -- -d $K:serialcomm=fast --scan
    # The line ends: api.c reads up to five lines for one answer.
    for b in 0 1; do
        emu_options setters=0 blank=$b
        emu_case "scan blank $b" --keep-header -- -d $K --scan
    done
    emu_options setters=0 blank=2
    emu_case "scan too many line ends" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 eol=cr blank=0
    emu_case "scan with CR" --keep-header -- -d $K --scan
    emu_options setters=0 eol=lf blank=0
    emu_case "scan with LF" --keep-header -- -d $K --scan
    emu_options setters=0 chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $K --scan
    emu_options setters=0 stall=line
    # The kernel takes the stall and the line stays as it was: the UART hears
    # nothing at 2400, and C waits as for a silent device.
    emu_case "line coding stalled" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan

    # MODEL: any LPS- will do, and the scan has the LPS-301 fixed.
    emu_options setters=0 model=LPS-
    emu_case "scan firmware 1.17 with no number" --keep-header -- -d $K --scan
    emu_options setters=0 model=LPS-305
    emu_case "scan another LPS is an LPS-301" --keep-header -- -d $K --scan
    emu_case "show another LPS has one channel" --keep-header -- -d $K --show
    emu_options setters=0 model=HP-1
    emu_case "scan another maker" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 model=lps-301
    emu_case "scan in lower case" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 model=ERROR
    emu_case "scan MODEL error" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 model=-
    emu_case "scan MODEL only OK" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 model=LPS-$LONG
    emu_case "scan MODEL too long" --keep-header -- -d $K --scan
    # Six line ends use up the five tries, and the second loop's OK comes with
    # no data, so the answer is none.
    emu_options setters=0 blank=0 model='|||LPS-301'
    emu_case "scan MODEL after five empty lines" --keep-header --no-output -- -d $K --scan
    # No OK within the second loop's five lines.
    emu_options setters=0 blank=0 model='LPS-301|a|b|c|d|e|f'
    emu_case "scan MODEL with no OK in time" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 blank=0 model='LPS-301|a'
    emu_case "scan MODEL with a line ahead of OK" --keep-header -- -d $K --scan
    emu_options setters=0 blank=0 model='LPS-301|a|b'
    emu_case "scan MODEL with two lines ahead of OK" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 model=
    emu_case "scan MODEL an empty line" --keep-header --no-output -- -d $K --scan

    # VERSION: firmware 1.17 is said to fail it while the output is on, which
    # is no error for the scan. What the device says then is a guess.
    emu_options setters=0 version=ERROR
    emu_case "scan VERSION error" --stderr -- -d $K --scan
    emu_case "show VERSION error" --keep-header -- -d $K --show
    emu_options setters=0 version=-
    emu_case "scan VERSION only OK" --stderr -- -d $K --scan
    emu_options setters=0 version=Ver-
    emu_case "scan VERSION empty" --keep-header -- -d $K --scan
    emu_options setters=0 version=Ver-1.17
    emu_case "scan VERSION 1.17" --keep-header -- -d $K --scan
    emu_options setters=0 version=Ver-$LONG
    emu_case "scan VERSION too long" --keep-header -- -d $K --scan
    emu_options setters=0 version=V1.0
    emu_case "scan VERSION unknown form" --keep-header --no-output -- -d $K --scan

    # STATUS: bit 6 is the output.
    emu_options setters=0 status=abc
    emu_case "scan status garbage" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 status=ERROR
    emu_case "scan status error" --stderr -- -d $K --scan
    emu_options setters=0 status=-
    emu_case "scan status only OK" --stderr -- -d $K --scan
    emu_options setters=0 status=64x
    emu_case "scan status with a tail" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 status=0x40
    emu_case "scan status in hex" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 status=2147483648
    emu_case "scan status above an int" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 status=2147483647
    emu_case "get enabled the largest status" -- -d $K -g CG1 --get enabled
    emu_options setters=0 status=-1
    emu_case "get enabled status -1" -- -d $K -g CG1 --get enabled
    emu_options setters=0 status=+64
    emu_case "get enabled status with a plus" -- -d $K -g CG1 --get enabled
    emu_options setters=0 status=191
    emu_case "get enabled every bit but the output" -- -d $K -g CG1 --get enabled
    emu_options setters=0 status=64
    emu_case "get enabled bit 6 alone" -- -d $K -g CG1 --get enabled

    # Nothing back: C's second read has no time left, and 0 is no timeout.
    emu_options setters=0 model=~
    emu_case "scan silent" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0 uart=9600/8n1
    emu_case "scan at the wrong rate" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0
    emu_case "serialcomm does not fit" --hangs --no-output --timeout 2 -- -d $K:serialcomm=9600/8n1 --scan
    emu_options setters=0 model=LPS-301^
    emu_case "scan MODEL with no OK" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0 model=LPS-301~
    emu_case "scan MODEL with no line end" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0 version=~
    emu_case "scan VERSION silent" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0 status=~
    emu_case "scan status silent" --hangs --no-output --timeout 2 -- -d $K --scan
    emu_options setters=0 in-stall=3
    emu_case "a stalled read" --keep-header -- -d $K --scan

    # --show and --get: the status from the scan and what was set since; the
    # readings stay 0 until a capture has read them. Nothing on the wire.
    emu_options setters=0 output=1 v1=5.000 i1=0.2500
    emu_case "show" --keep-header -- -d $K --show
    emu_case "show CG1" --keep-header -- -d $K -g CG1 --show
    emu_case "show no such group" --keep-header --expect-fail --stderr -- -d $K -g CG2 --show
    for key in channel_config limit_samples limit_time; do
        emu_case "get $key" -- -d $K --get $key
    done
    for key in voltage voltage_target current current_limit enabled; do
        emu_case "get CG1 $key" -- -d $K -g CG1 --get $key
    done
    emu_case "get voltage with no group" --expect-fail --stderr -- -d $K --get voltage
    emu_case "get enabled with no group" --expect-fail --stderr -- -d $K --get enabled
    emu_case "get channel_config on a group" --expect-fail --stderr -- -d $K -g CG1 --get channel_config
    emu_case "get conn" --expect-fail --stderr -- -d $K --get conn
    emu_case "get on another group" --expect-fail --stderr -- -d $K -g CG3 --get voltage
    emu_options setters=0
    emu_case "get CG1 enabled off" -- -d $K -g CG1 --get enabled

    # A setting: one line and its OK.
    emu_options
    emu_case "set enabled on" --no-output -- -d $K -g CG1 -c enabled=on --set
    emu_case "set enabled off when off" --no-output -- -d $K -g CG1 -c enabled=off --set
    emu_case "set then get enabled" -- -d $K -g CG1 -c enabled=on --get enabled
    emu_case "set voltage" --no-output -- -d $K -g CG1 -c voltage_target=5.5 --set
    emu_case "set current" --no-output -- -d $K -g CG1 -c current_limit=0.25 --set
    emu_case "set voltage max" --no-output -- -d $K -g CG1 -c voltage_target=32 --set
    emu_case "set voltage min" --no-output -- -d $K -g CG1 -c voltage_target=0 --set
    emu_case "set current max" --no-output -- -d $K -g CG1 -c current_limit=2 --set
    emu_case "set current listed min" --no-output -- -d $K -g CG1 -c current_limit=0.005 --set
    emu_case "set current under the listed min" --no-output -- -d $K -g CG1 -c current_limit=0.001 --set
    emu_case "set current 0" --no-output -- -d $K -g CG1 -c current_limit=0 --set
    emu_case "set voltage rounds" --no-output -- -d $K -g CG1 -c voltage_target=1.2345 --set
    emu_case "set voltage rounds in single precision" --no-output -- -d $K -g CG1 -c voltage_target=0.1235 --set
    emu_case "set voltage ten" --no-output -- -d $K -g CG1 -c voltage_target=10 --set
    emu_case "set then get voltage" -- -d $K -g CG1 -c voltage_target=3.3 --get voltage_target
    emu_case "set then get current" -- -d $K -g CG1 -c current_limit=0.1 --get current_limit
    emu_case "set voltage too high" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c voltage_target=32.01 --set
    emu_case "set voltage negative" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c voltage_target=-1 --set
    emu_case "set current too high" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c current_limit=2.01 --set
    emu_case "set current negative" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c current_limit=-1 --set
    emu_case "set voltage nan" --no-output -- -d $K -g CG1 -c voltage_target=nan --set
    emu_case "set current nan" --no-output -- -d $K -g CG1 -c current_limit=nan --set
    emu_case "set voltage minus 0" --no-output -- -d $K -g CG1 -c voltage_target=-0 --set
    emu_case "set current minus 0" --no-output -- -d $K -g CG1 -c current_limit=-0 --set
    emu_case "set voltage with no group" --stderr --expect-fail -- -d $K -c voltage_target=1 --set
    emu_case "set enabled with no group" --stderr --expect-fail -- -d $K -c enabled=on --set
    emu_case "set channel_config on a group" --stderr --expect-fail -- -d $K -g CG1 -c channel_config=Independent --set
    # Only the first mode for this model: api.c takes the others and does nothing.
    emu_case "set channel_config Independent" --no-output -- -d $K -c channel_config=Independent --set
    emu_case "set channel_config Track1" --no-output -- -d $K -c channel_config=Track1 --set
    emu_case "set then get channel_config Track2" -- -d $K -c channel_config=Track2 --get channel_config
    emu_case "set channel_config no such mode" --stderr --filter "$A" --expect-fail -- -d $K -c channel_config=Track3 --set
    emu_case "set several in hash order" --no-output -- -d $K -g CG1 -c current_limit=0.5:voltage_target=3.3 --set
    emu_options output=1
    emu_case "set enabled off" --no-output -- -d $K -g CG1 -c enabled=off --set
    emu_case "set enabled on when on" --no-output -- -d $K -g CG1 -c enabled=on --set
    emu_case "set then get enabled off" -- -d $K -g CG1 -c enabled=off --get enabled
    emu_options
    # The output never goes on ahead of its set points.
    emu_case "output goes on after the set point" --no-output -- -d $K -g CG1 -c enabled=on -c channel_group=CG1:voltage_target=3.3 --set
    emu_options output=1
    emu_case "output goes off first" --no-output -- -d $K -g CG1 -c voltage_target=3.3 -c channel_group=CG1:enabled=off --set
    emu_options
    emu_case "output with a voltage too high" --expect-fail --stderr --filter "$A" -- -d $K -g CG1 -c enabled=on -c channel_group=CG1:voltage_target=99 --set
    # A setting the device answers wrongly fails.
    emu_options set=ERROR
    emu_case "set answered ERROR" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c voltage_target=1 --set
    emu_case "enabled answered ERROR" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c enabled=on --set
    emu_options set=x
    emu_case "set answered with data" --stderr --filter "$A" --expect-fail -- -d $K -g CG1 -c current_limit=1 --set
    emu_options set=~
    emu_case "set answered nothing" --hangs --no-output --timeout 2 -- -d $K -g CG1 -c voltage_target=1 --set

    # A capture: VOUT1, IOUT1, STATUS in turn, each once the last has its OK, a
    # sample at each STATUS. When a limit ends it in the pass that read the OK,
    # api.c has one more request away before it notices.
    emu_options setters=0 output=1 v1=5.000
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 3 analog" -- -d $K --samples 3 -O analog
    emu_case "config limit_samples" -- -d $K -c limit_samples=2
    emu_case "config limit_samples 0" --stderr --expect-fail -- -d $K -c limit_samples=0 --samples 1
    emu_case "samples csv" -- -d $K --samples 2 -O csv:header=false:time=false
    emu_case "channel CH1" -- -d $K -C CH1 --samples 1
    emu_case "no such channel" --stderr --expect-fail -- -d $K -C CH2 --samples 1
    # Time decides the rounds, so both are kept up to the first round's end.
    Q1='/^bulk_out 02 494f5554310d0a/q'
    emu_case "time" --filter '3,$d' --tx-filter "$Q1" -- -d $K --time 500ms
    # A limit shorter than the first pass, which still sends its request.
    emu_case "time shorter than the first poll" --no-output -- -d $K --time 1ms
    emu_options setters=0 output=1 eol=cr blank=0
    emu_case "samples with CR" -- -d $K --samples 2
    emu_options setters=0 output=1 eol=lf blank=0
    emu_case "samples with LF" -- -d $K --samples 2
    emu_options setters=0 output=1 blank=0
    emu_case "samples blank 0" -- -d $K --samples 2
    # The output goes on ahead of the capture, set points first.
    emu_options output=0
    emu_case "capture with the output on" -- -d $K -g CG1 -c enabled=on --samples 1
    emu_case "capture with a setting" -- -d $K -g CG1 -c voltage_target=4 --samples 1
    emu_case "capture: the output on after the set point" -- -d $K -g CG1 -c enabled=on -c channel_group=CG1:voltage_target=4 --samples 1
    # Readings of every shape the strict conversion takes.
    emu_options setters=0 vout1=12.345,1,1.5e1,-2,+.5,5.,0x10 iout1=0.2500,0,1e-1,-0.5,.25,1.,0x1p-1
    emu_case "readings in turn" -- -d $K --samples 7 -O analog
    # A line that fits, by the 47 bytes the callback's buffer holds.
    emu_options setters=0 vout1=00000000000000000000000000000000000000000000001
    emu_case "a reading of 47 bytes" -- -d $K --samples 1
    # Data no one asked for, between a reading and its OK.
    emu_options setters=0 vout1='1.000|junk' iout1=0.5000
    emu_case "chatter" --stderr -- -d $K --samples 2
    emu_options setters=0 status=512,abc
    emu_case "status garbage in a capture" --stderr -- -d $K --samples 1
    emu_case "status garbage in a capture, its sample" -- -d $K --samples 1
    # What is no number: a logged error and 0.
    emu_options setters=0 vout1=1.000,ERROR,2.000
    emu_case "a reading says ERROR" -- -d $K --samples 3
    emu_options setters=0 iout1=0.5000,1.5V,0.2500
    emu_case "a reading has a unit" -- -d $K --samples 3
    emu_options setters=0 vout1=1e999,1.000
    emu_case "a reading above a double" -- -d $K --samples 1
    emu_options setters=0 vout1=1e-999,1.000
    emu_case "a reading below a double" -- -d $K --samples 1

    # What C never gets over. req_sent_at is real time and the timeout's check
    # takes monotonic time from it, so a request never expires.
    emu_options setters=0 answers=3
    emu_case "silent from the capture on" --hangs --no-output --timeout 2 -- -d $K --samples 1
    emu_options setters=0 answers=4
    emu_case "silent after the first reading" --hangs --no-output --timeout 2 -- -d $K --samples 1
    emu_options setters=0 vout1=1.000^
    emu_case "a reading with no OK" --hangs --no-output --timeout 2 -- -d $K --samples 1
    emu_options setters=0 vout1=-,1.000
    emu_case "a reading only OK" --hangs --no-output --timeout 2 -- -d $K --samples 1
    # A status with no number still sends the sample, and then no OK comes.
    emu_options setters=0 status=512,-
    emu_case "a status only OK" -- -d $K --samples 1
    emu_case "a status only OK, and on" --hangs --stderr --timeout 2 -- -d $K --samples 2
    emu_options setters=0 vout1=000000000000000000000000000000000000000000000001,1.000
    emu_case "a reading of 48 bytes" --hangs --no-output --timeout 2 -- -d $K --samples 1
    emu_options setters=0 vout1=$LONG$LONG,1.000
    emu_case "a reading too long" --hangs --no-output --timeout 2 -- -d $K --samples 1
    # A time limit does end those: every pass checks it.
    emu_options setters=0 answers=3
    emu_case "silent from the capture on, time" --no-output --tx-filter '/^bulk_out 02 564f555431/q' -- -d $K --time 300ms

    emu_options setters=0 answers=3
    emu_case "unplug in a capture" --hangs --no-output --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 1
    emu_options setters=0 answers=2
    emu_case "unplug in the scan" --hangs --no-output --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=motech-lps-301:conn=$EMU_TTY
}
