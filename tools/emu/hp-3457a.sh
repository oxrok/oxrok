# An HP 3457A multimeter (emu/src/models/hp-3457a.rs) on a socket alone: a real one
# has HP-IB, which oxrok does not. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a SCPI model".
EMU_MODEL=hp-3457a
EMU_TCP=only
EMU_DRIVER=hp-3457a
EMU_GCOV=hp-3457a
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    T=hp-3457a:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    emu_case "scan" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $T --show
    emu_case "get nplc" -- -d $T --get nplc
    emu_case "scan with no conn" --no-output -- -d hp-3457a --scan
    emu_case "tcp nothing listens" --no-output -- -d hp-3457a:conn=tcp-raw/127.0.0.1/1 --scan

    # Who it says it is.
    emu_options id=HP3478A
    emu_case "another id" --no-output -- -d $T --scan
    emu_options rev=2
    emu_case "rev one number" --keep-header -- -d $T --scan
    emu_options rev=x
    emu_case "rev no number" --stderr --expect-fail -- -d $T --scan
    emu_options rev=1.9,x
    emu_case "rev one good" --keep-header -- -d $T --scan
    emu_options rev=2.9,7.9,1
    emu_case "rev three numbers" --keep-header -- -d $T --scan
    emu_options rev=
    emu_case "rev empty" --keep-header -- -d $T --scan
    emu_options rev=-1.5,_8.0
    emu_case "rev signs and spaces" --keep-header -- -d $T --scan
    emu_options eol=lf
    emu_case "lines end in lf" --keep-header -- -d $T --samples 1 -O analog
    emu_options eol=none
    emu_case "lines with no end" --keep-header -- -d $T --samples 1 -O analog

    # What is behind it.
    emu_options card=0
    emu_case "show rear terminals" --keep-header -- -d $T --show
    emu_options card=44491
    emu_case "show 44491" --keep-header -- -d $T --show
    emu_case "scan 44491" --keep-header -- -d $T --scan
    emu_options card=44492
    emu_case "scan 44492" --keep-header -- -d $T --scan
    emu_options card=44493
    emu_case "scan unknown card" --keep-header -- -d $T --scan
    emu_options opt=
    emu_case "scan opt empty" --keep-header -- -d $T --scan
    emu_options opt=inf
    emu_case "scan opt infinite" --keep-header -- -d $T --scan
    emu_options rev=nan,inf
    emu_case "rev not finite" --keep-header -- -d $T --scan
    emu_options opt=x
    emu_case "scan opt no number" --keep-header -- -d $T --scan
    emu_options opt=44491.9
    emu_case "scan opt with a fraction" --keep-header -- -d $T --scan
    emu_options

    # Settings: what goes out, and what comes back.
    for _hp in 10 15 25 0.1 0.15 100 0.000002 1.5e1; do
        emu_case "set nplc $_hp" -- -d $T -c nplc=$_hp --get nplc
    done
    emu_case "set nplc over" --stderr --expect-fail -- -d $T -c nplc=101 --set
    emu_case "set nplc under" --stderr --expect-fail -- -d $T -c nplc=0.0000001 --set
    emu_case "set nplc at the floor" --stderr --expect-fail -- -d $T -c nplc=0.000001 --set
    emu_case "set nplc to nan" --no-output --stderr -- -d $T -c nplc=nan --set
    emu_options plcs=+1.000000E+00,x
    emu_case "nplc does not read back" -- -d $T -c nplc=10 --get nplc
    emu_options plcs=x
    emu_case "nplc does not read at open" -- -d $T --get nplc
    emu_options plcs=
    emu_case "nplc reads back empty at open" -- -d $T --get nplc
    emu_options plc=10
    emu_case "a preset of 10 plc is high resolution" -- -d $T --samples 2 -O analog
    emu_options

    for _hp in voltage voltage/dc voltage/ac voltage/ac/dc current current/ac current/ac/dc \
        resistance resistance/four_wire frequency frequency/ac; do
        emu_case "mq $_hp" -- -d $T -c measured_quantity=$_hp --samples 2 -O analog
    done
    emu_case "mq temperature" --stderr --expect-fail -- -d $T -c measured_quantity=temperature --set
    emu_case "mq voltage hold" --stderr --expect-fail -- -d $T -c measured_quantity=voltage/hold --set
    emu_case "mq resistance ac" --stderr --expect-fail -- -d $T -c measured_quantity=resistance/ac --set
    emu_case "mq twice the same flags" --no-output -- -d $T -c measured_quantity=voltage -c measured_quantity=voltage --set
    emu_case "mq twice other flags" -- -d $T -c measured_quantity=voltage -c measured_quantity=voltage/ac --samples 1 -O analog
    emu_case "mq twice another mq" -- -d $T -c measured_quantity=voltage -c measured_quantity=current --samples 1 -O analog

    # A capture.
    emu_case "capture" -- -d $T --samples 3 -O analog
    emu_case "capture csv" -- -d $T -c measured_quantity=voltage --samples 3 -O csv:header=false
    emu_case "capture csv no quantity" -- -d $T --samples 2 -O csv:header=false
    emu_case "capture high resolution" -- -d $T -c nplc=10 -c measured_quantity=voltage --samples 3 -O analog
    emu_case "capture 100 plc" -- -d $T -c nplc=100 --samples 2 -O analog
    emu_case "time is no limit here" --stderr --expect-fail -- -d $T --time 100 -O analog
    emu_case "continuous crashes C" --stderr --expect-fail -- -d $T --continuous -O analog

    # Digits, and what a reading may be.
    hp_values() {
        _hpn=$1
        shift
        emu_options "$@"
        emu_case "values $_hpn" -- -d $T -c measured_quantity=voltage --samples 8 -O analog
    }
    hp_values "small" readings=+9.999999E+00,+1.000000E+01,+1.000001E+01,+9.999990E-01,+1.000000E+00,+1.000000E-03,+1.234567E-07,+3.000000E+01
    hp_values "negative" readings=-9.999999E+00,-1.000000E+01,-1.000001E+01,-9.999990E-01,-1.000000E+00,-1.000000E-03,-1.234567E-07,-3.000000E+01
    hp_values "zero and overload" readings=+0.000000E+00,-0.000000E+00,+1.000000E+38,-1.000000E+38,+9.900000E+37,1e-300,1,5.
    hp_values "ranges" readings=+1.0E+00,+1.0E+00,+1.0E+00,+1.0E+00,+1.0E-02,-1.0E+00,+1.0E+00,+1.0E+00 ranges=0,+3.0E-01,+3.0E-03,+3.0E+06,+3.0E-03,0,-3,x
    emu_options readings=_1.5,+2.5E+00
    emu_case "a reading with a leading space" -- -d $T --samples 2 -O analog
    emu_options readings=1.5_,+2.5E+00
    emu_case "a reading with a trailing space" -- -d $T --samples 1 -O analog
    emu_options readings=x,+2.5E+00
    emu_case "a reading no number" -- -d $T --samples 1 -O analog
    emu_options readings=,+2.5E+00
    emu_case "a reading empty" -- -d $T --samples 2 -O analog
    emu_options readings=0x10,inf,nan,1e999,+2.5E+00
    emu_case "a reading of what strtod takes" -- -d $T --samples 3 -O analog
    emu_options ranges=x
    emu_case "a range no number" -- -d $T --samples 1 -O analog
    emu_options hires=x
    emu_case "a hires no number" -- -d $T -c nplc=10 --samples 1 -O analog
    emu_options hires=-2.0E-07
    emu_case "a hires below" -- -d $T -c nplc=10 --samples 1 -O analog
    emu_options

    # Terminals and cards.
    emu_options card=0
    emu_case "rear terminals" -- -d $T -C rear --samples 2 -O analog
    emu_case "front with a rear" -- -d $T --samples 1 -O analog
    emu_options card=44491
    emu_case "card one channel" -- -d $T -C 44491a3 --samples 2 -O analog
    emu_case "card three channels" -- -d $T -C 44491a0,44491a5,44491a13 --samples 25 -O analog
    emu_case "card two channels sync" -- -d $T -C 44491a1,44491a2 --samples 10 -O analog
    emu_case "front and card" --stderr --expect-fail -- -d $T -C Front,44491a3 --samples 1 -O analog
    emu_options card=44491 chan-skew=1
    emu_case "card out of sync" --stderr -- -d $T -C 44491a0,44491a1 --samples 25 -O analog
    emu_options card=44491 chans=x
    emu_case "card sync no number" --stderr -- -d $T -C 44491a0,44491a1 --samples 25 -O analog
    emu_options card=44491 chans=+1.500000E+00
    emu_case "card sync a fraction" --stderr -- -d $T -C 44491a0,44491a1 --samples 25 -O analog
    emu_options card=44492
    emu_case "card 44492" -- -d $T -C 44492a0,44492a9 --samples 3 -O analog
    emu_options

    # Pace: the real rate, reckoned from the integration time.
    emu_options real-rate=1
    emu_case "real rate 1 plc" -- -d $T --samples 3 -O analog
    emu_case "real rate 10 plc" -- -d $T -c nplc=10 --samples 2 -O analog
    emu_case "real rate 100 plc" -- -d $T -c nplc=100 --samples 1 -O analog
    emu_options

    # Silence and an end of the line.
    emu_options mute-at=1
    emu_case "scan silent" --hangs --no-output --timeout 4 -- -d $T --scan
    emu_options mute-at=3
    emu_case "rev silent" --hangs --no-output --timeout 4 -- -d $T --scan
    emu_options mute-at=4
    emu_case "nplc silent at open" --hangs --no-output --timeout 4 -- -d $T --get nplc
    emu_options mute-at=5
    emu_case "capture silent" --hangs --no-output --timeout 4 -- -d $T --samples 1 -O analog
    emu_options
}
