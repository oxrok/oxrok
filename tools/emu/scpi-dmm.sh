# SCPI multimeters (emu/src/models/scpi-dmm.rs), on USBTMC and on TCP. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=scpi-dmm
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=scpi-dmm
EMU_GCOV=scpi-dmm
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=scpi-dmm
    T=scpi-dmm:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    # Each name of the table, on the cheaper carrier.
    for m in 34405A 34410A 34460A GDM8251A GDM8255A GDM9060 GDM9061 34401A 34465A XDM1041 XDM2041 SDM3055; do
        # An OWON's *OPC? goes unanswered, which C can't take on TCP: see below.
        case $m in XDM*) emu_options model=$m opc=1 ;; *) emu_options model=$m ;; esac
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "capture $m" -- -d $T --samples 2 -O analog
    done
    emu_options
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "capture" -- -d $U --samples 2 -O analog
    emu_case "capture one" -- -d $T --samples 1 -O analog
    emu_case "time limit" -- -d $T --time 1 -O analog
    emu_case "csv" -- -d $T --samples 2 -O csv:header=false
    emu_case "get conn tcp" -- -d $T --get conn
    emu_case "get conn" -- -d $U --get conn
    emu_case "get measured quantity" -- -d $T --get measured_quantity
    emu_case "get limit samples" -- -d $T --get limit_samples
    emu_case "get limit time" -- -d $T --get limit_time
    emu_case "get range" -- -d $T --get range
    for m in 34410A GDM8251A GDM9060 34401A XDM2041 SDM3055 34465A; do
        case $m in XDM*) emu_options model=$m opc=1 ;; *) emu_options model=$m ;; esac
        emu_case "show $m" --keep-header -- -d $T --show
        emu_case "get measured quantity $m" -- -d $T --get measured_quantity
        emu_case "set measured quantity $m" --no-output -- -d $T -c measured_quantity=voltage/ac --set
    done
    emu_options model=34401A
    emu_case "34401A has no range" --no-output --expect-fail -- -d $T --get range
    emu_case "34401A sets no range" --no-output --expect-fail -- -d $T -c range=10 --set

    # Every function: what it asks, how it says it, and the unit. A 34460A for the
    # 34405A's table, with 10 ms of delay where a 34405A takes 2.5 s.
    for fn in VOLT:DC VOLT:AC CURR:DC CURR:AC RES FRES CONT CAP DIOD TEMP FREQ; do
        emu_options model=34460A func=$fn
        emu_case "$fn" -- -d $T --samples 1 -O analog
        emu_case "get measured quantity $fn" -- -d $T --get measured_quantity
        case $fn in
            CONT | DIOD | TEMP | FREQ) emu_case "get range $fn" --no-output --expect-fail -- -d $T --get range ;;
            *) emu_case "get range $fn" -- -d $T --get range ;;
        esac
    done
    emu_options model=34405A func=FREQ
    emu_case "34405A delays a measurement" -- -d $T --samples 1 -O analog
    emu_options model=34405A
    emu_case "34405A delays a function" --no-output -- -d $T -c measured_quantity=voltage/ac --set
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
        emu_case "reading $3" --no-output -- -d $T --samples 2 -O analog
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
    r 34465A "" empty
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
    r GDM8251A "" "gw empty"
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
        emu_case "function $3" --no-output -- -d $T --samples 1 -O analog
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
    emu_case "function unknown to the table in a capture" --no-output -- -d $T --samples 1 -O analog
    emu_options model=34465A 'functions="VOLT_+1E+02,+1E-03"|"VOLT_+1E+02,+1E-03"|'
    emu_case "function empty in a capture" --no-output -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A 'functions=01|01|"01"'
    emu_case "function gw in quotes in a capture" -- -d $T --samples 1 -O analog
    # At the start: refused.
    emu_options model=34465A functions='"VOLT:DC"'
    emu_case "function unknown at the start" --no-output --expect-fail -- -d $T --samples 1 -O analog
    emu_options model=34465A functions=
    emu_case "function empty at the start" --no-output --expect-fail -- -d $T --samples 1 -O analog
    emu_options model=GDM8251A functions=1
    emu_case "function gw one digit" --no-output --expect-fail -- -d $T --samples 1 -O analog
    unset -f q qn
    emu_options model=34465A functions=x
    emu_case "get measured quantity unknown" --no-output --expect-fail -- -d $T --get measured_quantity
    emu_options model=34465A functions=
    emu_case "get measured quantity empty" --no-output --expect-fail -- -d $T --get measured_quantity
    emu_case "show measured quantity empty" -- -d $T --show

    # Ranges.
    emu_options model=34465A manual=1
    emu_case "get range manual" -- -d $T --get range
    emu_options model=34465A manual=1 range=+1.00000E+01,+1.00000E-04
    emu_case "get range with a precision" -- -d $T --get range
    emu_options model=34465A manual=1 range=x
    emu_case "get range garbage" --no-output --expect-fail -- -d $T --get range
    emu_options model=34465A manual=1 range=
    emu_case "get range empty" --no-output --expect-fail -- -d $T --get range
    emu_options model=34465A auto=x
    emu_case "get range auto garbage" --no-output --expect-fail -- -d $T --get range
    emu_options model=34465A
    emu_case "set range" --no-output -- -d $T -c range=10 --set
    emu_case "set range auto" --no-output -- -d $T -c range=AuTo --set
    emu_case "set range refused" --no-output -- -d $T -c range=2000 --set
    emu_case "set range words" --no-output -- -d $T -c range=volts --set
    emu_options model=34465A func=TEMP
    emu_case "set range on no range" --stderr --expect-fail -- -d $T -c range=10 --set
    emu_options model=34405A func=VOLT:AC
    emu_case "34405A set range delays" --no-output -- -d $T -c range=10 --set

    # Settings ahead of a capture, and a function it does not have.
    emu_options model=34465A
    emu_case "set a function and capture" -- -d $T -c measured_quantity=resistance --samples 1 -O analog
    emu_case "set four wire" --no-output -- -d $T -c measured_quantity=resistance/four_wire --set
    emu_case "set dc diode" --no-output -- -d $T -c measured_quantity=voltage/dc/diode --set
    emu_case "set not in the table" --stderr --expect-fail -- -d $T -c measured_quantity=power --set
    emu_options model=GDM8251A
    emu_case "set GW celsius or fahrenheit" --no-output -- -d $T -c measured_quantity=temperature --set

    # How a USBTMC device is found and held.
    emu_reset
    emu_options
    emu_case "usbtmc by address" --keep-header -- -d scpi-dmm:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header -- -d scpi-dmm:conn=usbtmc --scan
    emu_case "tcp nothing listens" --no-output -- -d scpi-dmm:conn=tcp-raw/127.0.0.1/1 --scan
    emu_case "usbtmc kernel driver bound" --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' -- -d $U --samples 1 -O analog

    # An OWON's *OPC? goes unanswered, on what can wait.
    emu_options model=XDM2041
    emu_case "owon scan usb" --keep-header -- -d $U --scan
    emu_case "owon capture usb" -- -d $U --samples 2 -O analog

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
    emu_case "scan late" --keep-header -- -d $U --scan
    emu_case "scan late tcp" --keep-header -- -d $T --scan
    emu_options opc-late=3
    emu_case "opc says 0 three times" -- -d $T --samples 1 -O analog

    # The real rate: a reading comes after half a second, or longer than a read waits.
    emu_options model=34465A pace=25
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
    emu_options model=34465A silent-at=17
    emu_case "silent second reading usb" -- -d $U --samples 2 -O analog
    emu_options model=34465A silent-at=8
    emu_case "silent opc usb" -- -d $U --samples 1 -O analog
    emu_options model=34465A late-at=8:60
    emu_case "late opc usb" -- -d $U --samples 1 -O analog
    emu_case "late opc tcp" -- -d $T --samples 1 -O analog
    emu_options model=34465A opc-late=3
    emu_case "opc zeros usb" -- -d $U --samples 1 -O analog

    # A query that goes silent on USBTMC, where C's read gives up: the 4th is the
    # start's function, the 6th its precision, the 10th a round's function.
    emu_options model=34465A silent-at=4
    emu_case "silent function at the start usb" --no-output --expect-fail -- -d $U --samples 1 -O analog
    emu_options model=34465A silent-at=6
    emu_case "silent precision usb" -- -d $U --samples 1 -O analog
    emu_options model=GDM8251A silent-at=6
    emu_case "silent rate usb" -- -d $U --samples 1 -O analog
    # A get's range: the auto query is the 6th, the range the 8th.
    emu_options model=34465A silent-at=6
    emu_case "silent range auto usb" --no-output --expect-fail -- -d $U --get range
    emu_options model=34465A manual=1 silent-at=8
    emu_case "silent range usb" --no-output --expect-fail -- -d $U --get range

    # USBTMC's faults in a capture.
    emu_options model=34465A bad-tag-at=8
    emu_case "wrong tag on an opc" -- -d $U --samples 1 -O analog
    emu_options model=34465A empty-at=8
    emu_case "empty transfer on an opc" -- -d $U --samples 1 -O analog
    emu_options model=34465A message=4
    emu_case "answers in pieces" -- -d $U --samples 2 -O analog
}
