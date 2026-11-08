# SCPI multimeters (emu/src/models/scpi-dmm.rs), on USBTMC and on TCP. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=scpi-dmm
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=scpi-dmm
EMU_FULL=$EMU_DRIVER
EMU_GCOV=scpi-dmm
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=scpi-dmm
    T=scpi-dmm:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    # A re-attach moves the address; the form of a USBTMC conn stays compared.
    ADDR='s#usbtmc/[0-9]+\.[0-9]+#usbtmc/BUS.ADDR#'
    # One-sided: C's error for a key's value is "not applicable", oxrok's says why.
    NA="s/^(Failed to set device option .*: )not supported: .*$/\\1not applicable./"

    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block (scpi_tcp.c), and its recv() has no timeout, so sr_scpi_get_data's bound never runs and a silent instrument holds sigrok-cli for ever; oxrok's read has the timeout"
    SIGPIPE="sr_tcp_write_bytes is a plain send() (tcp.c) and nothing in libsigrok or sigrok-cli ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli (exit 141); oxrok's write returns EPIPE and the capture fails"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    EMPTY_BUG="sr_atod_ascii (strutil.c:317-319) gets a whole conversion of an empty string, so a meter that answers nothing reads 0 with exit 0 (protocol.c:400, :575); oxrok has no reading"
    FIRST_TIMEOUT="protocol.c:395-397 ends the capture at the first read's timeout, exit 0, where the meter may only be slow; oxrok asks or reads on within a bound"
    # sigrok-cli's logger exits at the critical of a refused start, ahead of the close
    # oxrok sends (*OPC? and SYST:LOC, which is what a meter wants): the tail goes, up to
    # the session's stop, from the comparison. Unit test: a_refused_start_closes_with_local.
    NOCLOSE_TCP='/^tcp_out 41424f5254/{s/^tcp_out 41424f52540a.*/tcp_out 41424f52540a/;q;}'
    NOCLOSE_USB='/^bulk_out 01 [0-9a-f]+41424f52540a/q'
    NOCLOSE_GW='/^tcp_out 434f4e463a535441543a46554e433f0a/{n;q;}'
    # sigrok-cli's close after --show, --get and --set sends nothing (api.c:453 returns
    # as device.c:779 has set the device inactive), so its meter stays remote. oxrok sends
    # *OPC? and SYST:LOC: those three lines go from both tools' logs, and so do the scan's
    # alike. A --set's last command shares a line with oxrok's *OPC?, so a set is an
    # --oracle-differs case instead. Pinned: "show says local", "get says local usb".
    LOCAL_TCP='/^tcp_out 2a4f50433f0a -> 6$/{
N
N
/\ntcp_out 535953543a4c4f430a -> 9$/d
}'
    LOCAL_USB='/^bulk_out 01 [0-9a-f]{8}06000000010000002a4f50433f0a0000 -> 20$/{
N
N
N
/\nbulk_out 01 [0-9a-f]{8}0900000001000000535953543a4c4f430a000000 -> 24$/d
}'
    LOCAL_BUG="api.c:453 returns at sdi->status <= SR_ST_INACTIVE, which sr_dev_close has just set (device.c:779), so the close after --show, --get and --set sends no SYST:LOC and the meter's panel stays locked, against commit 118da66f; oxrok sends *OPC? and SYST:LOC"
    RANGE_TEXT="config_set sends a range as it comes, in CONF:%s %s (protocol.c:242), so 10;*RST resets the meter; oxrok takes a number or AUTO, MIN, MAX, DEF and refuses the rest before the wire"
    # C asks again on USBTMC after a timeout, with a new request while the first is out,
    # which a real meter answers with the first one's answer under the old tag ("USBTMC
    # invalid bulk in header" in e90551c3 and 18426d1c): those cases take one-request=0,
    # the shell's leave to do so. oxrok waits out its one request, and after giving up
    # on a question clears the device, which C never does.
    NOWAIT="sr_scpi_get_data's timeout (scpi.c:281-306) leaves the request out, and the next read makes another (scpi_usbtmc_libusb.c), which a real meter answers under the old tag; oxrok reads again for its one request, and clears the device when it gives up on a question"
    # The clear's two requests, where sigrok-cli fails and no --oracle-differs can hold
    # the case: pinned in the NOWAIT cases.
    CLEAR='/^ctl_in 21 0[56] /d'
    RESYNC="protocol.c:122 and :308 end the capture at a function answer the table doesn't know, exit 0 with nothing; oxrok takes the line for out of step, lets it go quiet and starts the round again, three times at most"
    # A split, doubled or junk answer: the 15th query is a 34465A's second function
    # query, the 17th its second FETCH?.
    SHORT_READ="scpi_tcp_raw_read_data takes what one recv() gives (scpi_tcp.c), and scpi_tcp_raw_read_complete calls a short read the answer's end, so an answer the stack hands over in two is two answers: the first half is taken for the whole and every answer after it is one behind; oxrok's tcp-raw ends a text answer at its line feed"
    JUNK="after an answer that makes no reading C goes straight to its stop, whose *OPC? reads the real answer behind the junk; oxrok lets the line go quiet first"

    # Each name of the table, on the cheaper carrier.
    for m in 34405A 34410A 34460A GDM8251A GDM8255A GDM9060 GDM9061 34401A 34465A XDM1041 XDM2041 SDM3055; do
        # An OWON's *OPC? goes unanswered, which C can't take on TCP: see below.
        case $m in XDM*) emu_options model=$m opc=1 ;; *) emu_options model=$m ;; esac
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "capture $m" -- -d $T --samples 2 -O analog
    done
    emu_options
    emu_case "scan" --keep-header --filter "$ADDR" -- -d $U --scan
    emu_case "show" --tx-filter "$LOCAL_USB" --keep-header --filter "$ADDR" -- -d $U --show
    emu_case "show tcp" --tx-filter "$LOCAL_TCP" --keep-header -- -d $T --show
    emu_case "capture" --record -- -d $U --samples 2 -O analog
    emu_case "capture one" -- -d $T --samples 1 -O analog
    emu_case "time limit" -- -d $T --time 1 -O analog
    emu_case "csv" -- -d $T --samples 2 -O csv:header=false
    emu_case "get conn tcp" --tx-filter "$LOCAL_TCP" -- -d $T --get conn
    emu_case "get conn" --tx-filter "$LOCAL_USB" --filter "$ADDR" -- -d $U --get conn
    emu_case "get measured quantity" --tx-filter "$LOCAL_TCP" -- -d $T --get measured_quantity
    emu_case "get limit samples" --tx-filter "$LOCAL_TCP" -- -d $T --get limit_samples
    emu_case "get limit time" --tx-filter "$LOCAL_TCP" -- -d $T --get limit_time
    emu_case "get range" --tx-filter "$LOCAL_TCP" -- -d $T --get range
    for m in 34410A GDM8251A GDM9060 34401A XDM2041 SDM3055 34465A; do
        case $m in XDM*) emu_options model=$m opc=1 ;; *) emu_options model=$m ;; esac
        emu_case "show $m" --tx-filter "$LOCAL_TCP" --keep-header -- -d $T --show
        emu_case "get measured quantity $m" --tx-filter "$LOCAL_TCP" -- -d $T --get measured_quantity
        case $m in
            34410A | 34401A | SDM3055) emu_case "set measured quantity $m" --no-output -- -d $T -c measured_quantity=voltage/ac --set ;;
            *) emu_case "set measured quantity $m" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c measured_quantity=voltage/ac --set ;;
        esac
    done
    emu_options model=34401A
    emu_case "34401A has no range" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get range
    emu_case "34401A sets no range" --no-output --expect-fail --tx-filter "$LOCAL_TCP" -- -d $T -c range=10 --set

    # Every function: what it asks, how it says it, and the unit. A 34460A for the
    # 34405A's table, with 10 ms of delay where a 34405A takes 2.5 s.
    for fn in VOLT:DC VOLT:AC CURR:DC CURR:AC RES FRES CONT CAP DIOD TEMP FREQ; do
        emu_options model=34460A func=$fn
        emu_case "$fn" -- -d $T --samples 1 -O analog
        emu_case "get measured quantity $fn" --tx-filter "$LOCAL_TCP" -- -d $T --get measured_quantity
        case $fn in
            CONT | DIOD | TEMP | FREQ) emu_case "get range $fn" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get range ;;
            *) emu_case "get range $fn" --tx-filter "$LOCAL_TCP" -- -d $T --get range ;;
        esac
    done
    emu_options model=34405A func=FREQ
    emu_case "34405A delays a measurement" -- -d $T --samples 1 -O analog
    emu_options model=34405A
    emu_case "34405A delays a function" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c measured_quantity=voltage/ac --set
    emu_options model=34401A func=PER
    emu_case "34401A period" -- -d $T --samples 1 -O analog
    emu_options model=SDM3055 func=CAP
    emu_case "SDM3055 capacitance" -- -d $T --samples 1 -O analog
    for fn in VOLT:AC CURR:DC CURR:AC RES FRES CONT DIOD TEMP FREQ CAP; do
        emu_options model=XDM2041 opc=1 func=$fn
        emu_case "XDM2041 $fn" -- -d $T --samples 1 -O analog
    done
    for fn in VOLT:AC CURR:DC CURR:AC RES FRES CONT DIOD TEMP FREQ PER; do
        emu_options model=GDM8251A func=$fn
        emu_case "GDM8251A $fn" -- -d $T --samples 1 -O analog
    done
    for fn in VOLT:AC RES FRES CONT DIOD TEMP FREQ PER CAP; do
        emu_options model=GDM9060 func=$fn
        emu_case "GDM9060 $fn" -- -d $T --samples 1 -O analog
    done

    # A GW's other codes, and its overloads by code.
    emu_options model=GDM8251A functions="05|05"
    emu_case "GDM8251A code 05" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A functions="06|06"
    emu_case "GDM8251A code 06" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A functions="15|15|15"
    emu_case "GDM8251A fahrenheit" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A functions="09|09|09" readings=1.5E+01
    emu_case "GDM8251A celsius" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A func=RES readings=1.20000E+08
    emu_case "GDM8251A resistance OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A func=RES readings=1.19999E+08
    emu_case "GDM8251A resistance under OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8255A func=RES readings=1.20000E+08
    emu_case "GDM8255A resistance not OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8255A func=FRES readings=1.99999E+08
    emu_case "GDM8255A resistance OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A func=CONT readings=1.20000E+03
    emu_case "GDM8251A continuity OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A func=CONT readings=1.19000E+03
    emu_case "GDM8251A continuity under OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A func=DIOD readings=1.00000E+00
    emu_case "GDM8251A diode OL" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A func=DIOD readings=9.99999E-01
    emu_case "GDM8251A diode under OL" -- -d $T --samples 1 -O analog
    # The precision a GW names.
    for r in Slow Mid Fast Other; do
        emu_options model=GDM8251A rate=$r
        emu_case "GDM8251A rate $r" -- -d $T --samples 1 -O analog
    done
    emu_options model=GDM9060 rate=Fast
    emu_case "GDM9060 reads no rate" -- -d $T --samples 1 -O analog
    # Infinity.
    emu_options model=XDM2041 opc=1 readings=1E+9
    emu_case "XDM2041 overload" -- -d $T --samples 1 -O analog
    emu_options model=XDM2041 opc=1 readings=-1E+9
    emu_case "XDM2041 overload below" -- -d $T --samples 1 -O analog
    emu_options model=XDM2041 opc=1 readings=9.9E+8
    emu_case "XDM2041 under overload" -- -d $T --samples 1 -O analog

    # What a reading can say, on an instrument that goes on.
    r() {
        emu_options "model=$1" "readings=$2"
        emu_case "reading $3" -- -d $T --samples 2 -O analog
    }
    # C ends at these, with nothing to print.
    rn() {
        emu_options "model=$1" "readings=$2"
        emu_case "reading $3" --no-output --oxrok-fails "No usable reading" -- -d $T --samples 2 -O analog
    }
    r 34465A +9.90000000E+37 overload
    r 34465A -9.90000000E+37 "overload below"
    r 34465A +9.00000000E+37 "at the limit"
    r 34465A +9.00000001E+37 "over the limit"
    r 34465A __+1.5E+00__ "with spaces"
    r 34465A 1.5 "with no exponent"
    r 34465A .5 "with no integer"
    r 34465A 5. "with no fraction"
    r 34465A 5e-3 "in lower case"
    emu_options model=34465A readings=
    emu_case "reading empty" --no-output --oracle-differs "$EMPTY_BUG" --oxrok-fails "No usable reading" -- -d $T --samples 2 -O analog
    rn 34465A x garbage
    rn 34465A 1.5x "number and garbage"
    rn 34465A 1e "with no exponent digits"
    rn 34465A nan nan
    r 34465A inf inf
    r 34465A -infinity "minus infinity"
    rn 34465A 0x10 hexadecimal
    rn 34465A 1e999 "too large"
    rn 34465A 1e-999 "too small"
    rn 34465A 1e-310 "denormal"
    rn 34465A 0e99999999999 "zero with a long exponent"
    rn 34465A 1_2 "two numbers"
    rn 34465A 1,2 "two fields"
    emu_options model=GDM8251A readings=
    emu_case "reading gw empty" --no-output --oracle-differs "$EMPTY_BUG" --oxrok-fails "No usable reading" -- -d $T --samples 2 -O analog
    rn GDM8251A nan "gw nan"
    rn GDM8251A x "gw garbage"
    r GDM8251A inf "gw inf"
    unset -f r rn

    # What a function query can say: the start's two, and a round's.
    q() {
        emu_options "model=$1" "functions=$2|$2|$2"
        emu_case "function $3" -- -d $T --samples 1 -O analog
    }
    qn() {
        emu_options "model=$1" "functions=$2|$2|$2"
        emu_case "function $3" --no-output --oxrok-fails "No usable precision" -- -d $T --samples 1 -O analog
    }
    q 34465A '"RES_+1.00000E+08,+1.00000E+3"' "precision with a short exponent"
    q 34465A 'VOLT_+1E+02,+1E-03' "with no quotes"
    q 34465A '"RES_+1E+02,+1E-03' "with one quote"
    qn 34465A 'RES_+1E+02,+1E-03"' "with the other quote"
    qn 34465A '"FREQ_+1E+02,x"' "with a precision of garbage"
    q 34465A '"FREQ_+1E+02,1.5"' "with a precision of no exponent"
    q 34465A '"FREQ_+1E+02,1E"' "with an empty exponent"
    qn 34465A '"FREQ_+1E+02,1Ex"' "with an exponent of garbage"
    q 34465A '"FREQ_+1E+02,1E-200"' "with a large exponent"
    qn 34465A '"FREQ_+1E+02,1E-99999999999"' "with an exponent of no int"
    q 34465A '"FREQ_+1E+02,1.00000000000000000000E-06"' "with a long precision"
    q 34465A '"FREQ_+1E+02,_+1E-03"' "with a space in the precision"
    q 34465A '"FREQ_+1E+02,+1E-03_"' "with a space after the precision"
    q 34465A '"FREQ_"' "with no precision"
    q 34465A '"CONT"' "with no precision and the table's"
    q 34465A '"TEMP_THER,5000,+1.00000E+00,+1.00000E-01"' "with four fields"
    q GDM8251A 011 "gw three digits"
    q GDM8251A 150 "gw three digits, a temperature"
    q GDM9060 '"TEMP"' "906x no precision"
    emu_options model=34465A 'functions="VOLT_+1E+02,+1E-03"|"VOLT_+1E+02,+1E-03"|"PER_+1E+02,+1E-03"'
    emu_case "function unknown to the table in a capture" --oracle-differs "$RESYNC" -- -d $T --samples 1 -O analog
    emu_options model=34465A 'functions="VOLT_+1E+02,+1E-03"|"VOLT_+1E+02,+1E-03"|'
    emu_case "function empty in a capture" --oracle-differs "$RESYNC" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A 'functions=01|01|"01"'
    emu_case "function gw in quotes in a capture" -- -d $T --samples 1 -O analog
    # At the start: refused.
    emu_options model=34465A functions='"VOLT:DC"'
    emu_case "function unknown at the start" --no-output --expect-fail --tx-filter "$NOCLOSE_TCP" -- -d $T --samples 1 -O analog
    emu_options model=34465A functions=
    emu_case "function empty at the start" --no-output --expect-fail --tx-filter "$NOCLOSE_TCP" -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A functions=1
    emu_case "function gw one digit" --no-output --expect-fail --tx-filter "$NOCLOSE_GW" -- -d $T --samples 1 -O analog
    unset -f q qn
    emu_options model=34465A functions=x
    emu_case "get measured quantity unknown" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get measured_quantity
    emu_options model=34465A functions=
    emu_case "get measured quantity empty" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get measured_quantity
    emu_case "show measured quantity empty" --tx-filter "$LOCAL_TCP" -- -d $T --show

    # Ranges.
    emu_options model=34465A manual=1
    emu_case "get range manual" --tx-filter "$LOCAL_TCP" -- -d $T --get range
    emu_options model=34465A manual=1 range=+1.00000E+01,+1.00000E-04
    emu_case "get range with a precision" --tx-filter "$LOCAL_TCP" -- -d $T --get range
    emu_options model=34465A manual=1 range=x
    emu_case "get range garbage" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get range
    emu_options model=34465A manual=1 range=
    emu_case "get range empty" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get range
    emu_options model=34465A auto=x
    emu_case "get range auto garbage" --tx-filter "$LOCAL_TCP" --no-output --expect-fail -- -d $T --get range
    emu_options model=34465A
    emu_case "set range" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c range=10 --set
    emu_case "set range auto" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c range=AuTo --set
    emu_case "set range refused" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c range=2000 --set
    emu_case "set range words" --no-output --oracle-differs "$RANGE_TEXT" --expect-fail -- -d $T -c range=volts --set
    emu_case "set range with a command after it" --no-output --oracle-differs "$RANGE_TEXT" --expect-fail -- -d $T -c 'range=10;*RST' --set
    emu_case "set range with a command after it says why" --oracle-differs "$RANGE_TEXT" --expect-fail --stderr -- -d $T -c 'range=10;*RST' --set
    emu_options model=34465A func=TEMP
    emu_case "set range on no range" --stderr --expect-fail --filter "$NA" --tx-filter "$LOCAL_TCP" -- -d $T -c range=10 --set
    emu_options model=34405A func=VOLT:AC
    emu_case "34405A set range delays" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c range=10 --set

    # Settings ahead of a capture, and a function it does not have.
    emu_options model=34465A
    emu_case "set a function and capture" -- -d $T -c measured_quantity=resistance --samples 1 -O analog
    emu_case "set four wire" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c measured_quantity=resistance/four_wire --set
    emu_case "set dc diode" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c measured_quantity=voltage/dc/diode --set
    emu_case "set not in the table" --stderr --expect-fail --filter "$NA" --tx-filter "$LOCAL_TCP" -- -d $T -c measured_quantity=power --set
    emu_options model=GDM8251A
    emu_case "set GW celsius or fahrenheit" --no-output --oracle-differs "$LOCAL_BUG" -- -d $T -c measured_quantity=temperature --set

    # How a USBTMC device is found and held.
    emu_reset
    emu_options
    emu_case "usbtmc by address" --keep-header --filter "$ADDR" -- -d scpi-dmm:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header --filter "$ADDR" -- -d scpi-dmm:conn=usbtmc --scan
    emu_case "tcp nothing listens" --no-output -- -d scpi-dmm:conn=tcp-raw/127.0.0.1/1 --scan
    emu_case "usbtmc kernel driver bound" --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' -- -d $U --samples 1 -O analog

    # An OWON's *OPC? goes unanswered, on what can wait.
    emu_options model=XDM2041 one-request=0
    emu_case "owon scan usb" --keep-header --filter "$ADDR" --oracle-differs "$NOWAIT" -- -d $U --scan
    emu_case "owon capture usb" --oracle-differs "$NOWAIT" -- -d $U --samples 2 -O analog

    # What the instrument answers, and when.
    emu_options idn=Acme,X,1,2
    emu_case "scan another instrument" --no-output -- -d $U --scan
    emu_case "scan another instrument tcp" --no-output -- -d $T --scan
    emu_options say=Agilent,34405A,1,2
    emu_case "scan the short name" --keep-header -- -d $T --scan
    emu_options say=AGILENT_TECHNOLOGIES,34405A,1,2
    emu_case "scan the name in capitals" --keep-header -- -d $T --scan
    emu_options say=Agilent_Technologies,34405A,1
    emu_case "scan three fields" --keep-header -- -d $T --scan
    emu_options say=Agilent_Technologies,34405A
    emu_case "scan two fields" --no-output -- -d $T --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options silent-at=1
    emu_case "scan silent the first time" --no-output -- -d $U --scan
    emu_options model=34401A late-at=1:5
    emu_case "scan late" --keep-header --filter "$ADDR" -- -d $U --scan
    emu_case "scan late tcp" --keep-header -- -d $T --scan
    emu_options opc-late=3
    emu_case "opc says 0 three times" -- -d $T --samples 1 -O analog

    # The real rate: a reading comes after half a second, or longer than a read waits.
    emu_options model=34465A reading-pace=25
    emu_case "real rate" -- -d $T --samples 2 -O analog
    emu_case "real rate usb" -- -d $U --samples 2 -O analog
    emu_case "real rate, a time limit shorter than the pace" -- -d $T --time 100 -O analog
    emu_options model=34465A opc-pace=25
    emu_case "opc takes half a second after INIT" -- -d $T --samples 1 -O analog
    emu_case "opc takes half a second after INIT usb" -- -d $U --samples 1 -O analog

    # A reading late or silent, by count of queries: on a 34465A the 12th is the first
    # FETCH?, the 17th the second. 1.6 s is more than a read waits on USBTMC.
    emu_options model=34465A late-at=12:80
    emu_case "late reading tcp" -- -d $T --samples 2 -O analog
    emu_options model=34465A silent-at=17 one-request=0
    emu_case "silent second reading usb" --oracle-differs "$FIRST_TIMEOUT" -- -d $U --time 3000 -O analog
    emu_options model=34465A silent-at=8 one-request=0
    emu_case "silent opc usb" --oracle-differs "$NOWAIT" -- -d $U --samples 1 -O analog
    emu_options model=34465A late-at=8:60 one-request=0
    emu_case "late opc usb" --oracle-differs "$NOWAIT" -- -d $U --samples 1 -O analog
    emu_options model=34465A late-at=8:60
    emu_case "late opc tcp" -- -d $T --samples 1 -O analog
    emu_options model=34465A opc-late=3
    emu_case "opc zeros usb" -- -d $U --samples 1 -O analog

    # A query that goes silent on USBTMC, where C's read gives up: the 4th is the
    # start's function, the 6th its precision, the 10th a round's function.
    emu_options model=34465A silent-at=4 one-request=0
    emu_case "silent function at the start usb" --no-output --expect-fail --tx-filter "$CLEAR
$NOCLOSE_USB" -- -d $U --samples 1 -O analog
    emu_options model=34465A silent-at=6 one-request=0
    emu_case "silent precision usb" --oracle-differs "$NOWAIT" -- -d $U --samples 1 -O analog
    emu_options model=GDM8251A silent-at=6 one-request=0
    emu_case "silent rate usb" --oracle-differs "$NOWAIT" -- -d $U --samples 1 -O analog
    # A get's range: the auto query is the 6th, the range the 8th.
    emu_options model=34465A silent-at=6 one-request=0
    emu_case "silent range auto usb" --tx-filter "$LOCAL_USB
$CLEAR" --no-output --expect-fail -- -d $U --get range
    emu_options model=34465A manual=1 silent-at=8 one-request=0
    emu_case "silent range usb" --tx-filter "$LOCAL_USB
$CLEAR" --no-output --expect-fail -- -d $U --get range

    # USBTMC's faults in a capture.
    emu_options model=34465A bad-tag-at=8
    emu_case "wrong tag on an opc" -- -d $U --samples 1 -O analog
    emu_options model=34465A empty-at=8
    emu_case "empty transfer on an opc" -- -d $U --samples 1 -O analog
    emu_options model=34465A message=4
    emu_case "answers in pieces" -- -d $U --samples 2 -O analog

    # What C can't wait for, as the real ones are slow, and an end by a time limit: the
    # 12th query is the first FETCH? of a 34465A.
    emu_options model=34465A late-at=12:80 one-request=0
    emu_case "late reading usb" --oracle-differs "$FIRST_TIMEOUT" -- -d $U --samples 2 -O analog
    emu_options model=34465A silent-at=12 one-request=0
    emu_case "silent reading and a time limit usb" --no-output --oracle-differs "$NOWAIT" -- -d $U --time 200 -O analog
    emu_case "silent reading and a time limit warns" --oracle-differs "the warning is oxrok's: C says nothing of a capture with nothing in it" --stderr --filter '/^sr: /d' -- -d $U --time 200 -O analog
    emu_options model=34465A silent-at=12
    emu_case "silent reading and a time limit tcp" --no-output --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --time 200 -O analog
    emu_options model=34465A readings=x
    emu_case "garbage and a time limit" --no-output -- -d $T --time 5000 -O analog
    emu_case "garbage and a time limit warns" --oracle-differs "the warning is oxrok's: C says nothing of a capture with nothing in it" --stderr --filter '/^sr: /d' -- -d $T --time 5000 -O analog
    emu_options model=34465A 'readings=+1E+00|x'
    emu_case "garbage after a reading and a time limit" --oxrok-fails "No usable reading" -- -d $T --time 5000 -O analog

    # An OWON's *OPC? goes unanswered, on what can't wait.
    emu_options model=XDM2041
    emu_case "owon scan tcp" --keep-header --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --scan
    emu_case "owon capture tcp" --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --samples 1 -O analog

    # A silent scan on TCP, and an interrupt: the stop lands in a read that nothing ends.
    emu_options mute-at=1
    emu_case "scan silent tcp" --no-output --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --scan
    emu_options model=34465A mark-at=12 silent-at=13
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --timeout 8 --interrupt tcp-model -- -d $T --samples 100 -O analog

    # An unplug.
    emu_options model=34465A
    emu_case "unplug tcp" --oracle-bug "$SIGPIPE" --unplug tcp-in=12 --oxrok-fails "closed|unplug" -- -d $T --samples 5 -O analog
    emu_case "unplug usb" --unplug in=2:12 --oxrok-fails "unplug" -- -d $U --samples 5 -O analog

    # The close says local after a --show and a --get too: pinned here, filtered above.
    emu_options
    emu_case "show says local" --keep-header --oracle-differs "$LOCAL_BUG" -- -d $T --show
    emu_case "get says local usb" --oracle-differs "$LOCAL_BUG" -- -d $U --get measured_quantity

    # A function answer the table doesn't know, for good: the round starts again three
    # times, and then the capture fails.
    emu_options model=34465A 'functions="VOLT_+1E+02,+1E-03"|"VOLT_+1E+02,+1E-03"|"PER_+1E+02,+1E-03"|"PER_+1E+02,+1E-03"|"PER_+1E+02,+1E-03"|"PER_+1E+02,+1E-03"'
    emu_case "function unknown to the table for good" --no-output --oracle-differs "$RESYNC" --oxrok-fails "Unknown measurement quantity" -- -d $T --samples 1 -O analog

    # Answers that come in two, twice, late, or after junk.
    for carrier in tcp usb; do
        case $carrier in tcp) D=$T ;; *) D=$U ;; esac
        case $carrier in tcp) SPLIT="--oracle-differs" ;; *) SPLIT="" ;; esac
        emu_options model=34465A split-at=15:3
        emu_case "a function answer in two $carrier" ${SPLIT:+$SPLIT "$SHORT_READ"} -- -d $D --samples 3 -O analog
        emu_options model=34465A split-at=17:3
        emu_case "a reading in two $carrier" ${SPLIT:+$SPLIT "$SHORT_READ"} -- -d $D --samples 3 -O analog
        # USBTMC drops a second copy by its tag. On TCP whether sigrok-cli's next read
        # meets it or not goes with the clock, and a stall puts both copies in one read:
        # no stable case there until the shared fix of src/scpi/tcp.rs; a unit test,
        # a_reading_twice_on_a_line_with_no_tags, has it.
        case $carrier in
            usb)
                emu_options model=34465A double-at=17:1
                emu_case "a reading twice $carrier" -- -d $D --samples 3 -O analog
                ;;
        esac
        emu_options model=34465A junk-at=17:3
        case $carrier in
            tcp) emu_case "junk ahead of a reading $carrier" --oracle-differs "$JUNK" --oxrok-fails "No usable reading" -- -d $D --samples 3 -O analog ;;
            *) emu_case "junk ahead of a reading $carrier" --oxrok-fails "No usable reading" -- -d $D --samples 3 -O analog ;;
        esac
    done
    emu_options model=34465A late-at=15:80
    emu_case "a late function answer tcp" -- -d $T --samples 3 -O analog
}
