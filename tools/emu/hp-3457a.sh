# An HP 3457A multimeter (emu/src/models/hp-3457a.rs) on a socket alone: a real one
# has HP-IB, which oxrok does not. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a SCPI model".
EMU_MODEL=hp-3457a
EMU_TCP=only
EMU_DRIVER=hp-3457a
EMU_FULL=$EMU_DRIVER
EMU_GCOV=hp-3457a
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_tcp.c src/tcp.c"

# The port selects the terminals at the first capture after each open, and C takes the
# front for selected: oxrok's TERM FRONT goes from what is compared, and the counts with
# it, which the bytes say anyway. "a capture selects the front" holds it unmasked.
TF='s/5445524d2046524f4e540a//
s/^tcp_out ([0-9a-f]+) -> [0-9]+$/tcp_out \1/'
hp_case() {
    _hpc=$1
    shift
    emu_case "$_hpc" --tx-filter "$TF" "$@"
}

emu_cases() {
    T=hp-3457a:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    # oxrok's log prefix on its side only.
    L='s/^oxrok: (err|warn): oxrok::drivers::hp_3457a(::protocol)?: /sr: hp-3457a: /'
    # The core's count of a limit unmet: C never sends an End to count at.
    S="$L
/^oxrok: warn: oxrok::cli::pipeline: Device only sent 9 samples\\.$/d"
    # The C's sr_strerror alone, and oxrok's reason after it, which a unit test pins.
    BADSET="s/^(Failed to set device option '[a-z_]+': invalid argument): .*/\\1./
s/^(Failed to set device option '[a-z_]+'): not supported: .*/\\1: not applicable./"
    # What the core and the CLI say after a refused start, by name.
    REFUSED="$L
/^sr: (session|std): /d
/^free\\(\\): double free/d
/^timeout: the monitored command dumped core/d
/^Failed to start session\\.$/d"

    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block (scpi_tcp.c), and its recv() has no timeout, so sr_scpi_get_string's 1 s never runs and a silent meter holds sigrok-cli for ever; oxrok's read has the timeout"
    REV_CRITICAL="api.c:112 frees rev_numbers, which sr_scpi_get_floatv left NULL when no number came first: g_array_free's critical ends sigrok-cli; oxrok reports version 0.0, as api.c:103 says it means to"
    # lib a stop crashes C: api.c:406 frees active_channels at a stop, but the driver's
    # stop neither removes its source nor marks anything, so hp_3457a_receive_data goes
    # on and activate_next_channel reads the freed list. sigrok-cli does the same at
    # stdin's end. oxrok's stop ends the capture ("continuous then a stop", and a unit
    # test with answers coming).
    DOUBLE_FREE="api.c:370 frees active_channels at a refused start, and the session then stops the device (session.c:834), whose api.c:406 frees it again: free() aborts. oxrok refuses, and that is all"
    SPLIT="scpi_tcp_raw_read_data takes a short read for the end of a response (scpi_tcp.c), so each piece of an answer is an answer: the rest shifts every later one, and an empty piece measures again; oxrok reads an answer to its line end, as ID? showed it has one"
    SPLIT_CRLF="scpi_tcp_raw_read_data ends a response at a short read (scpi_tcp.c), so C takes the CR for the end, asks the next question, and reads the late LF at the head of that one's answer: out of step by a piece, which g_ascii_strtod's leading white space happens to forgive in this cut. oxrok waits for the line end"
    SPLIT_HANGS="$SPLIT; with an answer cut inside its number, C reads more answers than it asked for, and at the end waits on a blocking read for one that never comes"
    TERM_FRONT="devc->input_loc starts at 0, CONN_FRONT (api.c, g_malloc0), so hp_3457a_select_input sends no TERM FRONT and a meter left at the rear is read and labelled Front; oxrok sends TERM at the first capture after each open"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    TCP_EOF="a closed connection is a read of nothing in sr_tcp_read_bytes (tcp.c), which reaches sr_atod_ascii as an empty answer, so a 0: a false range that lets one more sample out, seen with this flag off as 2.2345669; then the next write's SIGPIPE ends sigrok-cli, exit 141. oxrok says the instrument closed the connection"
    EMPTY_IS_0="sr_atod_ascii (strutil.c) takes an empty string, as g_ascii_strtod stops at once with no error, so an empty answer reads as 0; oxrok takes it for no number and measures again"
    STRTOD="sr_atod_ascii's g_ascii_strtod reads hexadecimal, inf and nan, and passes them on as readings; oxrok takes them for no number and measures again"
    OPT_EMPTY="sr_atof_ascii reads an empty OPT? as 0 (protocol.c:146), which is the rear terminals' id; oxrok takes it for no card"
    OPT_INF="(unsigned int) of inf at protocol.c:149 is 0 on x86-64, the rear terminals' id; oxrok takes it for no number, so no card"
    REV_NAN="(int) of nan at api.c:108-109 is INT_MIN on x86-64; oxrok takes it for no number, so 0.0"
    REAR_SLIST="api.c:385 compares the card's id, 0 on the rear terminals, with the enum REAR_TERMINALS, 1, so C sends the terminals a scan list of channel -1 (,4294967295) and SADV AUTO; oxrok sends neither"
    SAME_MQ="protocol.c:117 compares the quantity alone, so voltage/ac after voltage says OK and sends nothing, and the samples stay DC; oxrok sends ACV and labels them AC"
    PLC_NAN="a NaN passes protocol.c:172's two comparisons, and %.0E makes it NPLC NAN; oxrok refuses it"
    PLC_UNKNOWN="protocol.c:182 drops NPLC?'s result, so a meter that doesn't state its time reads as the last value, or as 0 from dev_open (api.c:218), and hides the digits with no word; oxrok says the time isn't known"

    hp_case "scan" --keep-header -- -d $T --scan
    hp_case "show" --keep-header -- -d $T --show
    hp_case "get nplc" -- -d $T --get nplc
    hp_case "scan with no conn" --no-output -- -d hp-3457a --scan
    hp_case "tcp nothing listens" --no-output -- -d hp-3457a:conn=tcp-raw/127.0.0.1/1 --scan

    # Who it says it is.
    emu_options id=HP3478A
    hp_case "another id" --no-output -- -d $T --scan
    emu_options rev=2
    hp_case "rev one number" --keep-header -- -d $T --scan
    emu_options rev=x
    hp_case "rev no number" --oracle-bug "$REV_CRITICAL" --keep-header -- -d $T --scan
    emu_options rev=1.9,x
    hp_case "rev one good" --keep-header -- -d $T --scan
    emu_options rev=2.9,7.9,1
    hp_case "rev three numbers" --keep-header -- -d $T --scan
    emu_options rev=
    hp_case "rev empty" --keep-header -- -d $T --scan
    emu_options rev=-1.5,_8.0
    hp_case "rev signs and spaces" --keep-header -- -d $T --scan
    emu_options eol=lf
    hp_case "lines end in lf" --keep-header -- -d $T --samples 1 -O analog
    emu_options eol=none
    hp_case "lines with no end" --keep-header -- -d $T --samples 1 -O analog

    # What is behind it.
    emu_options card=0
    hp_case "show rear terminals" --keep-header -- -d $T --show
    emu_options card=44491
    hp_case "show 44491" --keep-header -- -d $T --show
    hp_case "scan 44491" --keep-header -- -d $T --scan
    emu_options card=44492
    hp_case "scan 44492" --keep-header -- -d $T --scan
    emu_options card=44493
    hp_case "scan unknown card" --keep-header -- -d $T --scan
    emu_options opt=
    hp_case "scan opt empty" --oracle-differs "$OPT_EMPTY" --keep-header -- -d $T --scan
    emu_options opt=inf
    hp_case "scan opt infinite" --oracle-differs "$OPT_INF" --keep-header -- -d $T --scan
    emu_options rev=nan,inf
    hp_case "rev not finite" --oracle-differs "$REV_NAN" --keep-header -- -d $T --scan
    emu_options opt=x
    hp_case "scan opt no number" --keep-header -- -d $T --scan
    emu_options opt=44491.9
    hp_case "scan opt with a fraction" --keep-header -- -d $T --scan
    emu_options

    # Settings: what goes out, and what comes back.
    for _hp in 10 15 25 0.1 0.15 100 0.000002 1.5e1; do
        hp_case "set nplc $_hp" -- -d $T -c nplc=$_hp --get nplc
    done
    hp_case "set nplc over" --stderr --expect-fail --filter "$BADSET" -- -d $T -c nplc=101 --set
    hp_case "set nplc under" --stderr --expect-fail --filter "$BADSET" -- -d $T -c nplc=0.0000001 --set
    hp_case "set nplc at the floor" --stderr --expect-fail --filter "$BADSET" -- -d $T -c nplc=0.000001 --set
    hp_case "set nplc to nan" --oracle-differs "$PLC_NAN" --stderr --expect-fail --filter "$BADSET" -- -d $T -c nplc=nan --set
    emu_options plcs=+1.000000E+00,x
    hp_case "nplc does not read back" --oracle-differs "$PLC_UNKNOWN" --stderr --expect-fail -- -d $T -c nplc=10 --get nplc
    emu_options plcs=x
    hp_case "nplc does not read at open" --oracle-differs "$PLC_UNKNOWN" --stderr --expect-fail --filter "$L" -- -d $T --get nplc
    emu_options plcs=
    hp_case "nplc reads back empty at open" --oracle-differs "$PLC_UNKNOWN" --stderr --expect-fail --filter "$L" -- -d $T --get nplc
    emu_options plc=10
    hp_case "a preset of 10 plc is high resolution" -- -d $T --samples 2 -O analog
    emu_options

    for _hp in voltage voltage/dc voltage/ac voltage/ac/dc current current/ac current/ac/dc \
        resistance resistance/four_wire frequency frequency/ac; do
        hp_case "mq $_hp" -- -d $T -c measured_quantity=$_hp --samples 2 -O analog
    done
    hp_case "mq temperature" --stderr --expect-fail --filter "$BADSET" -- -d $T -c measured_quantity=temperature --set
    hp_case "mq voltage hold" --stderr --expect-fail --filter "$BADSET" -- -d $T -c measured_quantity=voltage/hold --set
    hp_case "mq resistance ac" --stderr --expect-fail --filter "$BADSET" -- -d $T -c measured_quantity=resistance/ac --set
    hp_case "mq twice the same flags" --no-output -- -d $T -c measured_quantity=voltage -c measured_quantity=voltage --set
    hp_case "mq twice other flags" --oracle-differs "$SAME_MQ" -- -d $T -c measured_quantity=voltage -c measured_quantity=voltage/ac --samples 1 -O analog
    hp_case "mq twice another mq" -- -d $T -c measured_quantity=voltage -c measured_quantity=current --samples 1 -O analog

    # A capture.
    hp_case "capture" -- -d $T --samples 3 -O analog
    hp_case "capture csv" -- -d $T -c measured_quantity=voltage --samples 3 -O csv:header=false
    hp_case "capture csv no quantity" -- -d $T --samples 2 -O csv:header=false
    hp_case "capture high resolution" -- -d $T -c nplc=10 -c measured_quantity=voltage --samples 3 -O analog
    hp_case "capture 100 plc" -- -d $T -c nplc=100 --samples 2 -O analog
    hp_case "time is no limit here" --stderr --expect-fail -- -d $T --time 100 -O analog

    # Digits, and what a reading may be.
    hp_values() {
        _hpn=$1
        shift
        emu_options "$@"
        hp_case "values $_hpn" -- -d $T -c measured_quantity=voltage --samples 8 -O analog
    }
    hp_values "small" readings=+9.999999E+00,+1.000000E+01,+1.000001E+01,+9.999990E-01,+1.000000E+00,+1.000000E-03,+1.234567E-07,+3.000000E+01
    hp_values "negative" readings=-9.999999E+00,-1.000000E+01,-1.000001E+01,-9.999990E-01,-1.000000E+00,-1.000000E-03,-1.234567E-07,-3.000000E+01
    hp_values "zero and overload" readings=+0.000000E+00,-0.000000E+00,+1.000000E+38,-1.000000E+38,+9.900000E+37,1e-300,1,5.
    hp_values "ranges" readings=+1.0E+00,+1.0E+00,+1.0E+00,+1.0E+00,+1.0E-02,-1.0E+00,+1.0E+00,+1.0E+00 ranges=0,+3.0E-01,+3.0E-03,+3.0E+06,+3.0E-03,0,-3,x
    emu_options readings=_1.5,+2.5E+00
    hp_case "a reading with a leading space" -- -d $T --samples 2 -O analog
    emu_options readings=1.5_,+2.5E+00
    hp_case "a reading with a trailing space" -- -d $T --samples 1 -O analog
    emu_options readings=x,+2.5E+00
    hp_case "a reading no number" -- -d $T --samples 1 -O analog
    emu_options readings=,+2.5E+00
    hp_case "a reading empty" --oracle-differs "$EMPTY_IS_0" -- -d $T --samples 2 -O analog
    emu_options readings=0x10,inf,nan,1e999,+2.5E+00
    hp_case "a reading of what strtod takes" --oracle-differs "$STRTOD" -- -d $T --samples 3 -O analog
    emu_options ranges=x
    hp_case "a range no number" -- -d $T --samples 1 -O analog
    emu_options hires=x
    hp_case "a hires no number" -- -d $T -c nplc=10 --samples 1 -O analog
    emu_options hires=-2.0E-07
    hp_case "a hires below" -- -d $T -c nplc=10 --samples 1 -O analog
    emu_options

    # Terminals and cards.
    emu_options card=0
    hp_case "rear terminals" --oracle-differs "$REAR_SLIST" -- -d $T -C rear --samples 2 -O analog
    hp_case "front with a rear" -- -d $T --samples 1 -O analog
    emu_options card=44491
    hp_case "card one channel" -- -d $T -C 44491a3 --samples 2 -O analog
    hp_case "card three channels" -- -d $T -C 44491a0,44491a5,44491a13 --samples 25 -O analog
    hp_case "card two channels sync" -- -d $T -C 44491a1,44491a2 --samples 10 -O analog
    hp_case "front and card" --oracle-bug "$DOUBLE_FREE" --stderr --expect-fail --filter "$REFUSED" -- -d $T -C Front,44491a3 --samples 1 -O analog
    emu_options card=44491 chan-skew=1
    hp_case "card out of sync" --stderr --filter "$S" --oxrok-fails "out of sync" -- -d $T -C 44491a0,44491a1 --samples 25 -O analog
    emu_options card=44491 chans=x
    hp_case "card sync no number" --stderr --filter "$S" --oxrok-fails "synchronization" -- -d $T -C 44491a0,44491a1 --samples 25 -O analog
    emu_options card=44491 chans=+1.500000E+00
    hp_case "card sync a fraction" --stderr --filter "$S" --oxrok-fails "out of sync" -- -d $T -C 44491a0,44491a1 --samples 25 -O analog
    emu_options card=44492
    hp_case "card 44492" -- -d $T -C 44492a0,44492a9 --samples 3 -O analog
    emu_options

    # Pace: the real rate, reckoned from the integration time.
    emu_options real-rate=1
    hp_case "real rate 1 plc" -- -d $T --samples 3 -O analog
    hp_case "real rate 10 plc" -- -d $T -c nplc=10 --samples 2 -O analog
    hp_case "real rate 100 plc" -- -d $T -c nplc=100 --samples 1 -O analog
    emu_options

    # Answers in two pieces, from the open's NPLC? on: 200 ms apart, past any tail, and
    # 40 ms apart. sigrok-cli takes each piece for an answer.
    emu_options split=-1:10:4
    hp_case "cr and lf 200 ms apart" --oracle-differs "$SPLIT_CRLF" -- -d $T --samples 2 -O analog
    emu_options split=5:10:4
    hp_case "cut 5 bytes in 200 ms apart" --oracle-bug "$SPLIT_HANGS" --oracle-hangs --timeout 6 -- -d $T --samples 2 -O analog
    emu_options split=5:2:4
    hp_case "cut 5 bytes in 40 ms apart" --oracle-bug "$SPLIT_HANGS" --oracle-hangs --timeout 6 -- -d $T --samples 2 -O analog
    emu_options

    # Terminals: selected at a capture, which C takes the front for.
    emu_case "a capture selects the front" --oracle-differs "$TERM_FRONT" -- -d $T --samples 1 -O analog
    emu_options left-rear=1
    emu_case "terminals left at the rear" --oracle-differs "$TERM_FRONT" -- -d $T --samples 1 -O analog
    emu_options

    # Lateness, past the second oxrok's reads wait: the same wire.
    emu_options late-at=5:10
    hp_case "a reading 200 ms late" -- -d $T --samples 2 -O analog
    emu_options late-at=5:75
    hp_case "a reading 1.5 s late" -- -d $T --samples 2 -O analog
    emu_options late-at=2:75
    hp_case "id 1.5 s late" --keep-header -- -d $T --scan
    emu_options

    # Silence and an end of the line.
    emu_options mute-at=1
    # oxrok waits 30 s for ID?, so this costs about 65 s: it pins the warning.
    hp_case "scan silent" --stderr --filter "$L" --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 35 -- -d $T --scan
    emu_options mute-at=12 mark-at=12
    hp_case "continuous then a stop" --oracle-bug "$SIGINT_BUG" --interrupt tcp-model -- -d $T --continuous -O analog
    emu_options mute-at=6 mark-at=6
    hp_case "capture silent then a stop" --no-output --oracle-bug "$SIGINT_BUG" --interrupt tcp-model -- -d $T --samples 1 -O analog
    emu_options
    hp_case "unplug after a reading" --oracle-bug "$TCP_EOF" --unplug tcp-in=7 --oxrok-fails "closed" -- -d $T --samples 3 -O analog
    hp_case "unplug before a capture" --oracle-bug "$TCP_EOF" --no-output --unplug tcp-in=4 --oxrok-fails "closed" -- -d $T --samples 1 -O analog
}
