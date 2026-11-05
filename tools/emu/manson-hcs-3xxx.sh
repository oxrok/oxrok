# A Manson HCS-3xxx power supply (emu/src/models/manson-hcs-3xxx.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=manson-hcs-3xxx
EMU_CDC=1
EMU_DRIVER=manson-hcs-3xxx
EMU_FULL=$EMU_DRIVER
EMU_GCOV=manson-hcs-3xxx
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# What gcov leaves unreached, and why:
# - api.c:105-107: sigrok-cli passes only the options a driver lists.
# - api.c:163-164, protocol.c:168: hcs_parse_volt_curr_mode cannot fail.
# - api.c:197, 220-221, 277-278, 298, 304, 308-309: the core's checks of the
#   device and the key come first.
# - api.c:268-269: a SOUT that fails to go out needs the port to go between its
#   open and the first write. An unplug after the scan fails the open instead.
# - protocol.c:70: the arguments are constants.
# - protocol.c:75, 186: a read with no timeout on a tty that left does not
#   return (see "unplug in the scan").
# - protocol.c:216, 219: the callback always has its data.

emu_cases() {
    emu_conns
    # A whole reply: the text, its terminator and the OK line.
    ok() { emu_hex "$1\015OK\015"; }
    # oxrok's log prefix, on its side only, and what it adds after the error
    # sigrok-cli names.
    L='s/^oxrok: err: oxrok::drivers::manson_hcs_3xxx: /sr: manson-hcs-3xxx: /'
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    B='s/^(Failed to set device option .[a-z_]+.): device error: the device did not answer OK\.$/\1: generic\/unspecified error./'
    S="$L
$A
$B"
    SCAN_HANG="hcs_read_reply reads with serial_read_blocking's timeout 0 (protocol.c:73), which waits for ever, also on a tty that left; oxrok gives up after 2 s and finds no device"
    SET_HANG="config_set's hcs_read_reply (api.c:249, 260, 271) waits for ever for a reply; oxrok gives up after 2 s and fails the setting"
    NOREPLY_BUG="hcs_receive_data asks again every 500 ms for ever (protocol.c:239, 245), so a limit in samples never comes; oxrok ends a capture only a sample limit could end after 5 s with no reading"
    HUP_BUG="hcs_receive_data takes no revents but G_IO_IN as data (protocol.c:223) and ignores a failed send (protocol.c:245-246), so with the device gone it spins until a time limit; oxrok ends the capture"
    GARBAGE_BUG="hcs_parse_volt_curr_mode takes what makes no number for 0 (protocol.c:99, 105), so a garbled reply is a 0 V sample, and reads past a short line (protocol.c:104); oxrok drops the reply"
    SPLICE_BUG="hcs_receive_data keeps what came of a reply when it asks again (protocol.c:239), and it joins the next: 12 and 123601000 read as 12.12 V, 36.01 A on a 10 A supply; oxrok forgets it at the 500 ms"
    FORM_BUG="hcs_parse_volt_curr_mode takes whatever strtod makes of a field (protocol.c:99, 105), inf, nan, hex and all; oxrok takes the eight digits the wiki gives"
    INF_BUG="api.c:173-175 take inf for a limit, so api.c:244's two comparisons let an infinite target through and %03.0f sends VOLTinf; oxrok takes a limit that is not finite for 0"
    OVERRUN_BUG="handle_new_data has no bound (protocol.c:184): over 50 bytes with no OK it overwrites devc->buflen behind buf (protocol.h:78-79) and goes on from wherever that says, then reports made up values; oxrok drops the reply"
    NUL_BUG="handle_new_data's g_str_has_suffix (protocol.c:192) stops at a NUL, so no reply after it ends and the buffer runs over (protocol.c:184), with made up values; oxrok loses that reply alone"
    NAN_BUG="api.c:244 and 255's two comparisons let a NaN through, and %03.0f sends 'VOLTnan' to a live output; oxrok refuses it"
    ZERO_BUG="api.c:255 lets -0.0 through, and %03.0f sends it as CURR-00; oxrok sends CURR000"
    ORDER_BUG="sigrok-cli sets -c's options in hash table order (session.c:700-701), here SOUT0 before VOLT, so the output goes on at the old voltage; oxrok sets enabled=on last"
    PARTIAL_BUG="sigrok-cli sets -c's options one at a time in hash table order (session.c:700-707), so SOUT0 goes out before a later option is refused; oxrok checks them all first and sends nothing"
    LOOSE_BUG="sigrok-cli takes what is no value (session.c's opt_to_gvar, boolstring) and sends what it makes of it; oxrok refuses it and sends nothing"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # Every model code of api.c's table, with and without the prefix. A scan
    # asks GMOD, GETD, GMAX, and sets nothing: setters=0 flags a setting.
    for m in 3100 3102 3104 3150 3200 3202 3204 3300 3302 3304 3400 3402 3404 3600 3602 3604; do
        emu_options setters=0 model=$m
        emu_case "scan $m" --keep-header -- -d $K --scan
        emu_options setters=0 model=$m gmod=HCS-$m
        emu_case "scan HCS-$m" --keep-header -- -d $K --scan
    done

    # How far the ID check stretches (api.c:131-137).
    emu_options setters=0 gmod=ACME
    emu_case "scan an unknown ID" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options setters=0 gmod=hcs-3202
    emu_case "scan in lower case" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex 'OK\015OK\015')
    emu_case "scan ID OK" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex '\015OK\015')
    emu_case "scan an empty ID" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex '3202\015xOK\015')
    emu_case "scan OK at the end of a word" --keep-header -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex '3202\015ERR\015')
    emu_case "scan ID not OK" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex '3202\000\015OK\015')
    emu_case "scan ID with a NUL" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex '3202\015OK\015trailing')
    emu_case "scan bytes after the reply" --keep-header -- -d $K --scan
    # Filling the buffer: the loop takes up to 51, where reply[] holds 50.
    P45=012345678901234567890123456789012345678901234
    emu_options setters=0 gmod-raw=$(emu_hex "3202\015$P45\015")
    emu_case "scan ID 50 bytes no OK" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex "$P45\015OK\015")
    emu_case "scan ID 49 bytes" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex "${P45}12\015OK\015")
    emu_case "scan ID 51 bytes" --stderr --filter "$L" --keep-header -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex "${P45}123456789012345678901234567890\015OK\015")
    emu_case "scan ID too long" --keep-header --no-output -- -d $K --scan

    # The next two questions.
    emu_options setters=0 getd-raw=$(emu_hex 'ERR\015\015')
    emu_case "scan GETD not OK" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 gmax-raw=$(emu_hex 'ERR\015\015')
    emu_case "scan GMAX not OK" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 gmax-raw=$(ok xxxyyy)
    emu_case "show GMAX garbage" --keep-header -- -d $K --show
    # Short replies: api.c:173's &tokens[0][3] and protocol.c:104's tokens[0] + 4
    # read past the string, so what that finds is masked.
    PAST_GMAX='s/^(    current_limit: [^,]+, )[0-9.]+( \(current\))?,/\1past the end,/'
    PAST_GETD='s/^    current: .*$/    current: past the end/; s/^CH1: [^ ]+ A$/CH1: past the end A/'
    emu_options setters=0 gmax-raw=$(ok 36)
    emu_case "show GMAX short" --filter "$PAST_GMAX" --keep-header -- -d $K --show
    emu_options setters=0 gmax-raw=$(ok '')
    emu_case "show GMAX empty" --filter "$PAST_GMAX" --keep-header -- -d $K --show
    emu_options setters=0 gmax-raw=$(ok 360)
    emu_case "show GMAX no current" --keep-header -- -d $K --show
    emu_options setters=0 gmax-raw=$(ok 36.010.0)
    emu_case "show GMAX with points" --keep-header -- -d $K --show
    emu_options setters=0 vmax=362 imax=102
    emu_case "show GMAX above the rating" --keep-header -- -d $K --show
    emu_options setters=0 model=3104
    emu_case "show hundredths" --keep-header -- -d $K --show
    emu_options setters=0 model=3602
    emu_case "show 3602" --keep-header -- -d $K --show
    emu_options setters=0 getd-raw=$(ok xxxxyyyyz)
    emu_case "show GETD garbage" --keep-header -- -d $K --show
    emu_options setters=0 getd-raw=$(ok 12)
    emu_case "show GETD short" --oracle-differs "$GARBAGE_BUG" --filter "$PAST_GETD" --keep-header -- -d $K --show
    emu_options setters=0 getd-raw=$(ok 1234)
    emu_case "show GETD no current" --oracle-differs "$GARBAGE_BUG" --keep-header -- -d $K --show

    # Silence: serial_read_blocking's timeout of 0 waits for ever.
    emu_options setters=0 answers=0
    emu_case "scan no answer" --oracle-bug "$SCAN_HANG" --oracle-hangs --timeout 5 --keep-header --no-output -- -d $K --scan
    emu_options setters=0 answers=1
    emu_case "scan no GETD" --oracle-bug "$SCAN_HANG" --oracle-hangs --timeout 5 --keep-header --no-output -- -d $K --scan
    emu_options setters=0 answers=2
    emu_case "scan no GMAX" --oracle-bug "$SCAN_HANG" --oracle-hangs --timeout 5 --keep-header --no-output -- -d $K --scan
    emu_options setters=0 gmod-raw=$(emu_hex '3202\015')
    emu_case "scan one line of two" --oracle-bug "$SCAN_HANG" --oracle-hangs --timeout 5 --keep-header --no-output -- -d $K --scan

    emu_options setters=0
    emu_case "no such node" --keep-header --no-output -- -d manson-hcs-3xxx:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d manson-hcs-3xxx --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d manson-hcs-3xxx --show

    # The line: the UART must fit.
    emu_case "serialcomm as the default" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset" --oracle-bug "$SCAN_HANG" --oracle-hangs --timeout 5 --keep-header --no-output -- -d $K:serialcomm=19200/8n1 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses
    # them first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan

    # What --show and --get ask for: the scan's values, no more on the wire.
    emu_case "show" --keep-header -- -d $K --show
    emu_case "get conn" --stderr --expect-fail -- -d $K --get conn
    for key in voltage voltage_target current current_limit enabled limit_samples limit_time; do
        emu_case "get $key" -- -d $K --get $key
    done
    emu_options setters=0 output=1 load=10
    emu_case "show on, with a load" --keep-header -- -d $K --show
    emu_case "get voltage on" -- -d $K --get voltage
    emu_case "get current on" -- -d $K --get current
    emu_case "get enabled on" -- -d $K --get enabled
    # A short circuit: constant current at 0.00 V reads as off (protocol.c:113).
    emu_options setters=0 getd-raw=$(ok 000005001)
    emu_case "get enabled at 0 V in CC" -- -d $K --get enabled

    # A setting has a physical effect: each goes out as asked, alone, and
    # waits for its OK.
    emu_options
    emu_case "set voltage_target" --no-output -- -d $K -c voltage_target=5.0 --set
    emu_case "set voltage_target min" --no-output -- -d $K -c voltage_target=1 --set
    emu_case "set voltage_target max" --no-output -- -d $K -c voltage_target=36 --set
    emu_case "set voltage_target rounds" --no-output -- -d $K -c voltage_target=3.33 --set
    emu_case "set voltage_target rounds up" --no-output -- -d $K -c voltage_target=3.36 --set
    emu_case "set voltage_target half" --no-output -- -d $K -c voltage_target=3.25 --set
    emu_case "set voltage_target too low" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=0.9 --set
    emu_case "set voltage_target too high" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=36.1 --set
    emu_case "set voltage_target negative" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=-1 --set
    emu_case "set voltage_target nan" --oracle-differs "$NAN_BUG" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=nan --set
    emu_case "set voltage_target inf" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=inf --set
    emu_case "set current_limit" --no-output -- -d $K -c current_limit=2.5 --set
    emu_case "set current_limit 0" --no-output -- -d $K -c current_limit=0 --set
    emu_case "set current_limit max" --no-output -- -d $K -c current_limit=10 --set
    emu_case "set current_limit too high" --stderr --expect-fail --filter "$S" -- -d $K -c current_limit=10.1 --set
    emu_case "set current_limit negative zero" --oracle-differs "$ZERO_BUG" --no-output -- -d $K -c current_limit=-0 --set
    emu_case "set current_limit nan" --oracle-differs "$NAN_BUG" --stderr --expect-fail --filter "$S" -- -d $K -c current_limit=nan --set
    emu_case "set enabled on" --no-output -- -d $K -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -c enabled=off --set
    emu_case "set then read back" -- -d $K -c voltage_target=3.3 --get voltage_target
    emu_case "set current then read back" -- -d $K -c current_limit=2.5 --get current_limit
    emu_case "set enabled then read back" -- -d $K -c enabled=on --get enabled
    emu_case "show after a set" --keep-header -- -d $K -c voltage_target=3.3:current_limit=2.5:enabled=on --show
    emu_options model=3104
    emu_case "set hundredths" --no-output -- -d $K -c current_limit=1.23 --set
    emu_case "set hundredths max" --no-output -- -d $K -c current_limit=2.5 --set
    emu_case "set hundredths too high" --stderr --expect-fail --filter "$S" -- -d $K -c current_limit=2.51 --set
    # The maximum goes through a float (api.c:173-175).
    emu_options vmax=362 imax=102
    emu_case "set the GMAX of 36.2" --no-output -- -d $K -c voltage_target=36.2 --set
    emu_case "set the GMAX of 10.2" --no-output -- -d $K -c current_limit=10.2 --set
    emu_case "set over the GMAX" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=36.3 --set
    emu_options gmax-raw=$(ok xxxyyy)
    emu_case "set with GMAX garbage" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=5 --set
    emu_case "set current with GMAX garbage" --no-output -- -d $K -c current_limit=0 --set
    emu_options gmax-raw=$(ok infinf)
    emu_case "set inf with GMAX inf" --oracle-differs "$INF_BUG" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=inf --set
    # Together: the order is the question.
    emu_options
    emu_case "set in sigrok-cli's order" --no-output -- -d $K -c current_limit=0.5:voltage_target=3.3:enabled=on --set
    emu_case "set output before setpoints" --oracle-differs "$ORDER_BUG" --no-output -- -d $K -c enabled=on:voltage_target=3.3:current_limit=0.5 --set
    emu_case "output with a voltage too high" --oracle-bug "$PARTIAL_BUG" --stderr --expect-fail --filter "$S" -- -d $K -c enabled=on:voltage_target=99 --set
    emu_case "output with a voltage that is no number" --oracle-bug "$PARTIAL_BUG" --stderr --expect-fail -- -d $K -c enabled=on:voltage_target=abc --set
    emu_case "output with an unknown option" --oracle-bug "$PARTIAL_BUG" --stderr --expect-fail -- -d $K -c enabled=on:volage_target=5 --set
    emu_case "output with a malformed boolean" --oracle-differs "$LOOSE_BUG" --stderr --expect-fail -- -d $K -c enabled=1x --set

    # Settings the device refuses.
    emu_options ack-raw=$(emu_hex 'ERR\015')
    emu_case "voltage not OK" --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=5 --set
    emu_case "current not OK" --stderr --expect-fail --filter "$S" -- -d $K -c current_limit=5 --set
    emu_case "enabled not OK" --stderr --expect-fail --filter "$S" -- -d $K -c enabled=on --set
    emu_options ack-raw=$(emu_hex 'xOK\015')
    emu_case "ack with a prefix" --no-output -- -d $K -c voltage_target=5 --set
    emu_options answers=3
    emu_case "voltage no answer" --oracle-bug "$SET_HANG" --oracle-hangs --timeout 5 --stderr --expect-fail --filter "$S" -- -d $K -c voltage_target=5 --set
    emu_case "current no answer" --oracle-bug "$SET_HANG" --oracle-hangs --timeout 5 --stderr --expect-fail --filter "$S" -- -d $K -c current_limit=5 --set
    emu_case "enabled no answer" --oracle-bug "$SET_HANG" --oracle-hangs --timeout 5 --stderr --expect-fail --filter "$S" -- -d $K -c enabled=on --set
    emu_options ack-raw=$(emu_hex 'OK')
    emu_case "ack cut short" --oracle-bug "$SET_HANG" --oracle-hangs --timeout 5 --stderr --expect-fail --filter "$S" -- -d $K -c enabled=on --set
    emu_options load=10 ack-raw=$(emu_hex "${P45}123OK\015")
    emu_case "ack of 51 bytes then a capture" -- -d $K -c enabled=on --samples 2
    emu_options ack-raw=$(emu_hex "$P45$P45\015")
    emu_case "ack too long" --stderr --expect-fail --filter "$S" -- -d $K -c enabled=on --set

    # A capture: GETD, its reply, the next at once; a sample at each OK.
    emu_options setters=0 output=1 load=10
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 5" -- -d $K --samples 5
    emu_case "samples csv" -- -d $K --samples 3 -O csv:header=false:time=false
    emu_case "samples analog" --keep-header -- -d $K --samples 2 -O analog
    emu_case "channel CH1" -- -d $K -C CH1 --samples 2
    emu_case "no such channel" --stderr --expect-fail --filter 's/^Failed to set channels: invalid argument: //' -- -d $K -C CH2 --samples 2
    # hwdriver.c's check_key refuses it ahead of the driver, and logs that.
    Z="$A
/^sr: hwdriver: Cannot set/d"
    emu_case "config limit_samples" -- -d $K -c limit_samples=2 --get limit_samples
    emu_case "config limit_samples 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_samples=0 --samples 1
    emu_case "config limit_time 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_time=0 --samples 1
    emu_options setters=0 output=1 load=1 vset=120 iset=20
    emu_case "samples in CC" -- -d $K --samples 2
    emu_options setters=0 model=3104 output=1 load=10 vset=600 iset=250
    emu_case "samples at 60 V" -- -d $K --samples 1
    # A time limit: how many polls that is depends on the clock, so both are
    # kept up to the first reply of the capture.
    emu_options setters=0 output=1 load=10
    emu_case "time" --filter '3,$d' --tx-filter '/^bulk_in 82 13 -> /{x;s/^/x/;/^xxx/{x;q};x}' -- -d $K --time 100ms
    # Settings ahead of a capture.
    emu_options load=10
    emu_case "capture with the output on" -- -d $K -c enabled=on --samples 1
    emu_case "capture with setpoints" -- -d $K -c current_limit=1:voltage_target=5:enabled=on --samples 1
    emu_case "capture after a set" -- -d $K -c voltage_target=5 --samples 2

    # Replies in turn: readings, odd and wrong.
    emu_options setters=0 getd-raw=$(ok 123401000),$(ok 123501011),$(ok 000000000),$(ok 123601000)
    emu_case "readings in turn" -- -d $K --samples 5
    emu_case "readings in turn csv" -- -d $K --samples 5 -O csv:header=false:time=false
    emu_options setters=0 getd-raw=$(ok ' 12 100'),$(ok '1.2 1.0 0'),$(ok '\055100\0551000'),$(ok 1e0102e1),$(ok 123401000)
    emu_case "readings as strtod takes them" --oracle-differs "$FORM_BUG" -- -d $K --samples 5
    emu_options setters=0 getd-raw=$(ok infxinfx0),$(ok nanxnanx0),$(ok 0x100x100),$(ok 12340100)
    emu_case "readings inf nan hex" --oracle-differs "$FORM_BUG" -- -d $K --samples 4
    emu_options setters=0 getd-raw=$(ok 123401000),$(ok xxxxyyyyz),$(ok 123601000)
    emu_case "garbage reading" --oracle-differs "$GARBAGE_BUG" -- -d $K --samples 3
    emu_options setters=0 getd-raw=$(ok 123401000),$(ok 1234),$(ok 123601000)
    emu_case "reading with no current" --oracle-differs "$GARBAGE_BUG" -- -d $K --samples 3
    emu_options setters=0 getd-raw=$(ok 123401000),$(ok 12),$(ok 123601000)
    emu_case "reading cut short" --oracle-differs "$GARBAGE_BUG" --filter "$PAST_GETD" -- -d $K --samples 3
    emu_options setters=0 getd-raw=$(ok 123401000),$(emu_hex 'OK\015'),$(ok 123601000)
    emu_case "reading with only OK" --oracle-differs "$GARBAGE_BUG" --filter "$PAST_GETD" -- -d $K --samples 3
    emu_options setters=0 getd-raw=$(ok 123401000),$(emu_hex 'x\000OK\015'),$(ok 123601000)
    emu_case "reading with a NUL" --oracle-differs "$NUL_BUG" --timeout 15 -- -d $K --samples 3
    # A fragment and silence: it joins the next reply in C.
    emu_options setters=0 getd-raw=$(ok 123401000),$(ok 123401000),$(emu_hex '12'),$(ok 123601000)
    emu_case "a fragment of a reply" --oracle-differs "$SPLICE_BUG" -- -d $K --samples 3
    # Two at once: the next GETD meets a reply that is already there.
    emu_options setters=0 getd-raw=$(emu_hex '123401000\015OK\015123501000\015OK\015'),$(ok 123601000)
    emu_case "two replies at once" -- -d $K --samples 2
    # No terminator: 49 bytes just fit in buf[50].
    emu_options setters=0 getd-raw=$(ok 123401000),$(emu_hex "${P45}1234"),$(ok 123601000)
    emu_case "49 bytes with no OK" --oracle-differs "$OVERRUN_BUG" --timeout 8 -- -d $K --samples 3
    L1K=$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45$P45
    emu_options setters=0 getd-raw=$(ok 123401000),$(emu_hex "$L1K$L1K$L1K$L1K"),$(ok 123601000)
    emu_case "4 KB with no OK" --oracle-differs "$OVERRUN_BUG" --timeout 8 -- -d $K --samples 3

    # Nothing but garbage: C sends 0 V and is done, oxrok gives up after 5 s. How
    # many requests that is depends on the clock, so both are kept to the third.
    emu_options setters=0 getd-raw=$(ok xxxxyyyyz)
    emu_case "only garbage" --oracle-differs "$GARBAGE_BUG" --stderr --filter "$L
/^oxrok: warn: /d" --tx-filter '/^bulk_in 82 13 -> /{x;s/^/x/;/^xxx/{x;q};x}' --timeout 15 -- -d $K --samples 2

    # Silence: GETD again each 500 ms, for ever (protocol.c:239, 245).
    emu_options setters=0 answers=3
    # Their number depends on the clock: a unit test holds their pace.
    emu_case "silent from the capture on" --oracle-bug "$NOREPLY_BUG" --oracle-hangs --timeout 8 --no-output --tx-filter 's/^bulk_out 02 (474554440d)+ -> [0-9]+$/bulk_out 02 GETD repeated/' -- -d $K --samples 1
    emu_case "silent with a time limit" --no-output -- -d $K --time 700ms
    emu_options setters=0 answers=5
    emu_case "silent after two" --oracle-bug "$NOREPLY_BUG" --oracle-hangs --timeout 8 --tx-filter 's/^bulk_out 02 (474554440d)+ -> [0-9]+$/bulk_out 02 GETD repeated/' -- -d $K --samples 3

    # A gap of 6 s of silence, longer than the bound, then readings, then silence: a time
    # limit lets it go on as C does. Its last requests depend on the clock.
    emu_options setters=0 answers=17 getd-raw=$(ok 123401000),,,,,,,,,,,,,$(ok 123501000),$(ok 123601000)
    emu_case "silence then readings under a time limit" --timeout 15 --tx-filter '/^bulk_in 82 13 -> /{x;s/^/x/;/^xxx/{x;q};x}' -- -d $K --time 7s

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 output=1 chunk=1
    emu_case "one byte per transfer" -- -d $K --samples 2
    emu_options setters=0 output=1 in-stall=2
    emu_case "a stalled read" --keep-header -- -d $K --scan
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $K --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # An unplug during the scan, a setting and a capture.
    emu_options setters=0 answers=1
    emu_case "unplug in the scan" --oracle-bug "$SCAN_HANG" --oracle-hangs --unplug model --timeout 5 --keep-header --no-output -- -d $K --scan
    emu_options answers=3
    emu_case "unplug in a setting" --oracle-bug "$SET_HANG" --oracle-hangs --unplug model --timeout 5 --stderr --expect-fail --filter "$S" -- -d $K -c enabled=on --set
    # With --samples C never ends and logs two alternating lines as fast as it
    # can, 33 MB in 5 s, which no .oracle can hold: a time limit ends it, exit 0.
    W="$L
/^oxrok: warn: /d"
    emu_case "unplug in a capture" --oracle-differs "$HUP_BUG" --unplug model --timeout 10 --stderr --filter "$W" --tx-filter '/^unplug$/q' -- -d $K --time 2s
    # A quiet point: one GETD goes unanswered and the signal lands in its wait.
    # The next would be answered, and on: a capture that ignored the stop runs
    # into the timeout, and its next request shows in the .tx.
    emu_options setters=0 output=1 gap=5
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $K --continuous
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=manson-hcs-3xxx:conn=$EMU_TTY
}
