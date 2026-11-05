# An Agilent or Keysight U12xx meter behind its IR cable
# (emu/src/models/agilent-dmm.rs) on the CDC-ACM shell. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=agilent-dmm
EMU_CDC=1
EMU_DRIVER=agilent-dmm
EMU_FULL=$EMU_DRIVER
EMU_GCOV=agilent-dmm
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# TEXT with the meter's line end, as hex for an option. `%` is `%%`. Not
# emu_hex, whose printf takes a leading `-` for a flag.
ag_h() {
    printf -- "$1\r\n" | od -v -An -tx1 | tr -d ' \n'
}

# Each argument with its line end, as a list for an option.
ag_list() {
    _agl=
    for _agt in "$@"; do
        _agl=$_agl${_agl:+,}$(ag_h "$_agt")
    done
    printf %s "$_agl"
}

# One case per CONF answer: ag_confs MODEL ANSWER...
ag_confs() {
    _agm=$1
    shift
    for _agc in "$@"; do
        emu_options model=$_agm conf=$(ag_h "\"$_agc\"")
        emu_case "$_agm conf $_agc" -- -d $F -C P1 --samples 1
    done
}

# One case per log entry: ag_logs MODEL ENTRY...
ag_logs() {
    _agm=$1
    shift
    for _agc in "$@"; do
        emu_options model=$_agm log=$(ag_h "\"$_agc\"")
        emu_case "$_agm log $_agc" -- -d $F -C P1 -c data_source=Log-Hand --samples 1
    done
}

emu_cases() {
    emu_conns
    # Resolutions, as each family's CONF answer spells them.
    R6='+1.000000E+00,+1.000000E-04'
    R8='+1.00000000E+00,+1.00000000E-04'
    # oxrok's log prefix on its side only, then libserialport's lines, which
    # only sigrok-cli has, by name. Anything else, a panic or a CLI error,
    # stays and fails the case.
    L='s/^oxrok: (err|warn): oxrok::drivers::agilent_dmm(::protocol)?: /sr: agilent-dmm: /
/^sr: serial-libsp: /d'
    # A refused start: what the core says after the driver, by name, one line
    # of it with an address. The driver's own line stays.
    REFUSED="$L
/^sr: (session|std): /d
s/^not supported: (Log data .*)$/sr: agilent-dmm: \\1/
/^Failed to start session\\.$/d"
    # The C's sr_strerror alone, and oxrok's reason after it.
    BADSET="s/^(Failed to set device option '[a-z_]+': invalid argument): .*/\\1./"
    WEDGED_BUG="once a line fills the buffer with no LF, protocol.c:188's loop never reads again, and no sample limit can end the run; oxrok drops the line and reads the next"
    NUL_BUG="api.c:145-146: a line that is empty as a string splits to an empty vector, and strcmp gets tokens[0], NULL; oxrok finds no device"
    HUP_BUG="a port gone: protocol.c:226-228 logs a failed write, and the job's next try does the same, for ever; oxrok ends the capture"
    # Not a C bug: --oracle-bug --oracle-hangs is what the harness has for "C does
    # not end by itself". C's jobs retry, and pick up when the meter answers.
    SILENT_DIFFERS="a choice, not a C bug: C goes on asking a silent meter (protocol.c:112-116), and would pick up when it answers; oxrok ends a capture that only a sample limit could end, after 10 s, with an error"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    CSV_BUG="output/csv.c:713 dumps a row once channels_seen reaches channel_count, and csv.c:191 counts the disabled channels too, so with a display off rows are overwritten before they go out; oxrok writes them all"

    # The series, each with its identity and displays.
    for _agm in U1231A U1232A U1233A U1241A U1242A U1241B U1242B U1241C U1242C \
        U1251A U1252A U1253A U1251B U1252B U1253B \
        U1271A U1272A U1273A U1273AX U1281A U1282A; do
        emu_options model=$_agm
        emu_case "scan $_agm" --keep-header -- -d $F --scan
    done
    # A capture per family; the conf and log rows below go by family too.
    for _agm in U1232A U1242B U1242C U1253B U1272A U1282A; do
        emu_options model=$_agm
        emu_case "$_agm samples" -- -d $F --samples 3
    done
    emu_options
    emu_case "show" --keep-header -- -d $F --show
    emu_options model=U1232A
    emu_case "show one display" --keep-header -- -d $F --show
    emu_options model=U1242C
    emu_case "show two displays" --keep-header -- -d $F --show
    emu_options
    emu_case "get samplerate" --keep-header -- -d $F --get samplerate
    emu_case "get data_source" --keep-header -- -d $F --get data_source
    emu_case "get limit_samples" --keep-header -- -d $F --get limit_samples
    emu_case "get limit_time" --keep-header -- -d $F --get limit_time
    emu_case "get conn" --keep-header --stderr --expect-fail -- -d $F --get conn
    emu_case "set then get samplerate" --keep-header -- -d $F -c samplerate=10 --get samplerate
    emu_case "set then get data_source" --keep-header -- -d $F -c data_source=Log-Auto --get data_source
    emu_case "set then get limit_samples" --keep-header -- -d $F -c limit_samples=7 --get limit_samples
    emu_case "set then show" --keep-header -- -d $F -c samplerate=10 --show
    emu_case "set samplerate" --keep-header --no-output -- -d $F --set -c samplerate=20
    emu_case "samplerate too high" --keep-header --stderr --expect-fail --filter "$BADSET" -- -d $F --set -c samplerate=21
    emu_case "samplerate too low" --keep-header --stderr --expect-fail --filter "$BADSET" -- -d $F --set -c samplerate=0
    emu_case "data_source unknown" --keep-header --stderr --expect-fail --filter "$BADSET" -- -d $F --set -c data_source=Nope
    emu_case "no such node" --keep-header --no-output -- -d agilent-dmm:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d agilent-dmm --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d agilent-dmm --show

    # serialcomm= is the line, and 9600/8n1 otherwise.
    emu_case "serialcomm as the meter" --keep-header -- -d $F:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset" --keep-header --no-output -- -d $F:serialcomm=19200/8n1 --scan
    emu_options model=U1272A uart=19200/8n1
    emu_case "serialcomm as the U1272A" --keep-header -- -d $F:serialcomm=19200/8n1 --scan
    emu_case "U1272A at 19200" -- -d $F:serialcomm=19200/8n1 --samples 2
    emu_case "U1272A not at 9600" --keep-header --no-output -- -d $F --scan
    emu_options
    # sigrok-cli opens the port before it reads the parameters: the open's
    # lines are all that may differ.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $F:serialcomm=fast --scan

    # What a scan takes for an identity (api.c:141-180).
    emu_options id=$(ag_h 'Keysight Technologies,U1253B,x,V1')
    emu_case "id Keysight on an Agilent model" --keep-header -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1282A,x,V1')
    emu_case "id Agilent on a Keysight model" --keep-header -- -d $F --scan
    emu_options id=$(ag_h 'HP,U1253B,x,V1')
    emu_case "id another maker" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'agilent technologies,U1253B,x,V1')
    emu_case "id in lower case" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253B,x')
    emu_case "id no version" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253B')
    emu_case "id no serial" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies')
    emu_case "id maker alone" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253B,x,')
    emu_case "id empty version" --keep-header -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253B,,V1')
    emu_case "id empty serial" --keep-header -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253B,x,V1,and,more')
    emu_case "id commas in the version" --keep-header -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U9999A,x,V1')
    emu_case "id unknown model" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253,x,V1')
    emu_case "id model with less" --keep-header --no-output -- -d $F --scan
    emu_options id=$(ag_h 'Agilent Technologies,U1253BX,x,V1')
    emu_case "id model with more" --keep-header --no-output -- -d $F --scan
    emu_options id=$(emu_hex 'Agilent Technologies,U1253B,x,V1\n')
    emu_case "id ends in LF" --keep-header -- -d $F --scan
    emu_options id=$(emu_hex 'Agilent Technologies,U1253B,x,V1\r')
    emu_case "id ends in CR" --keep-header -- -d $F --scan
    # Time ends the read, and what came counts.
    emu_options id=$(emu_hex 'Agilent Technologies,U1253B,x,V1')
    emu_case "id no line end" --keep-header -- -d $F --scan
    emu_options id=$(emu_hex '\r\nAgilent Technologies,U1253B,x,V1\r\n')
    emu_case "id an empty line first" --keep-header --no-output -- -d $F --scan
    emu_options id=$(emu_hex 'x\r\n')
    emu_case "id garbage" --keep-header --no-output -- -d $F --scan
    # Up to 127 bytes, and the rest is not read.
    emu_options id=$(ag_h "Agilent Technologies,U1253B,x,$(printf 'v%.0s' $(seq 1 96))")
    emu_case "id of 126 bytes" --keep-header -- -d $F --scan
    emu_options id=$(ag_h "Agilent Technologies,U1253B,x,$(printf 'v%.0s' $(seq 1 200))")
    emu_case "id longer than the buffer" --keep-header -- -d $F --scan
    emu_options id=$(ag_h "Agilent Technologies,$(printf 'm%.0s' $(seq 1 200)),x,V1")
    emu_case "id model past the buffer" --keep-header --no-output -- -d $F --scan
    emu_options id=$(emu_hex 'Agilent Technologies,U1253B,x,V1\000more\r\n')
    emu_case "id cut at a NUL" --keep-header -- -d $F --scan
    emu_options id=none
    emu_case "id silence" --keep-header --no-output -- -d $F --scan

    # The fields family (recv_conf_u123x).
    ag_confs U1232A V,0,AC V,1,DC MV,2,DC MV,0,AC A,0,DC A,1,AC UA,1,DC UA,0,AC FREQ,1,AC FREQ,0,DC RES,1 RES,4 CAP,3 DIOD
    ag_confs U1272A V,0,ACDC MV,1,ACDC A,1,ACDC MA,2,DC MA,0,AC MA,1,ACDC UA,0,ACDC FREQ,1,ACDC RES,2 CAP,1 DIOD TEMP
    # Taken by its table and in no branch (protocol.c:824-826): no quantity.
    emu_options model=U1242B conf=$(ag_h '"T3:K CEL"')
    emu_case "U1242B T3" --no-output -- -d $F -C P1 --time 100ms
    # None of these is an answer to the family: the line is dropped, and a
    # time limit ends the capture before any sample.
    emu_options model=U1232A conf=$(ag_h '"MA,0,DC"')
    emu_case "U1232A has no MA" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1232A conf=$(ag_h '"TEMP"')
    emu_case "U1232A has no TEMP" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1232A conf=$(ag_h '"V,0,ACDC"')
    emu_case "U1232A has no ACDC" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1232A conf=$(ag_h '"V,10,DC"')
    emu_case "U1232A a resolution of two digits" --no-output -- -d $F -C P1 --time 100ms
    # The STAT bytes the family reads (recv_stat_u123x), and the ones it reads
    # into the next CONF, a second on: temperature on the mV position, and
    # continuity on the resistance one. A case that needs a second CONF runs at
    # 1 Hz, where each job's second starts after the one before it, so FETC is
    # due after CONF and STAT however late they are. At 5 Hz the fifth FETC can
    # come after the next CONF on a loaded box, and the output changes.
    for _ags in 100000000000000000000 010000000000000000000 001000000000000000000 000100000000000000000 000000010000000000000 000000000000000010000 1100xxxxxxxxxxxxxxxx0; do
        emu_options model=U1232A stat=$(ag_h "\"$_ags\"")
        emu_case "U1232A stat $_ags" -- -d $F -C P1 --samples 1
    done
    emu_options model=U1232A conf=$(ag_h '"MV,1,DC"') stat=$(ag_h '"000000010000000000000"')
    emu_case "U1232A temperature on the mV position" -- -d $F -C P1 -c samplerate=1 --samples 2
    emu_options model=U1232A conf=$(ag_h '"RES,1"') stat=$(ag_h '"000000000000000010000"')
    emu_case "U1232A continuity on the resistance position" -- -d $F -C P1 -c samplerate=1 --samples 2
    emu_options model=U1272A conf=$(ag_list '"RES,1"' '"RES,1"' '"DIOD"') stat=$(ag_h '"000000000000000010000"')
    emu_case "U1272A from continuity to diode" -- -d $F -C P1 -c samplerate=1 --samples 3

    # The other families (recv_conf_u124x_5x), each with what its table takes.
    ag_confs U1242B "VOLT $R6" "VOLT:AC $R6" "VOLT:DC $R6" "VOLT:ACDC $R6" "VOLT:CD $R6" "CURR $R6" "CURR:AC $R6" "CURR:DC $R6" "CURR:ACDC $R6" "CURR:D $R6" "RES $R6" "CAP $R6" "FREQ $R6" "CPER:4-20mA $R6" "CPER:0-20mA $R6" 'T1:K CEL' 'T2:J FAR' 'T1:K XX' 'TEMP:K CEL' 'TEMP:K FAR' DIOD
    ag_confs U1242B 'VOLT +1.0000000E+00,+1.0000000E-04' 'VOLT +1.00000000E+00,+1.00000000E-05'
    ag_confs U1253B "VOLT $R6" "VOLT:AC $R6" "VOLT:DC $R6" "VOLT:ACDC $R6" "CURR $R6" "CURR:AC $R6" "CURR:DC $R6" "CURR:ACDC $R6" "RES $R6" "CONT $R6" "COND $R6" "CAP $R6" "FREQ $R6" "CPER:4-20mA $R6" "PULS:PWID $R6" "PULS:PWID:AC $R6" 'T1:K CEL' 'T2:J FAR' 'TEMP:K CEL' 'TEMP:K FAR' DIOD PULS:PDUT
    ag_confs U1242C "VOLT $R8" "VOLT:AC $R8" "VOLT:HRAT $R8" "CURR $R8" "CURR:AC $R8" "RES $R8" "CONT $R8" "CAP $R8" "FREQ $R8" "FREQ:AC $R8" "CPER:4-20mA $R8" 'TEMP:K CEL' 'TEMP:K FAR' 'NCV HI' 'NCV LO' DIOD TEMP
    ag_confs U1282A "VOLT $R8" "VOLT:AC $R8" "VOLT:DC $R8" "VOLT:ACDC $R8" "CURR $R8" "CURR:AC $R8" "CURR:DC $R8" "CURR:ACDC $R8" "RES $R8" "CONT $R8" "COND $R8" "CAP $R8" "FREQ $R8" "FREQ:AC $R8" "FC1 $R8" "FC100 $R8" "CPER:0-20mA $R8" "PULS:PWID $R8" "PULS:PWID:DC $R8" 'TEMP:K CEL' 'TEMP:K FAR' 'NCV HIGH' 'NCV LOW' DIOD PULS:PDUT
    # A resolution's exponent: above and far below the point.
    ag_confs U1253B 'VOLT +1.000000E+00,+1.000000E+01' 'VOLT:AC +1.000000E+00,+1.000000E-09' 'PULS:PWID +1.000000E+00,+1.000000E-09'
    # PULS:NDUT is in U125x's table and in no branch (protocol.c:824-826), and
    # the first CONF leaves the display with no quantity, so no sample.
    emu_options model=U1253B conf=$(ag_h '"PULS:NDUT"')
    emu_case "U1253B PULS:NDUT" --no-output -- -d $F -C P1 --time 100ms
    # A second answer in a turn of the dial: PULS:NDUT keeps what came before.
    emu_options model=U1253B conf=$(ag_list "\"VOLT:AC $R6\"" '"PULS:NDUT"')
    emu_case "U1253B PULS:NDUT keeps the last" -- -d $F -C P1 -c samplerate=1 --samples 2
    for _agm in U1242B U1253B U1242C U1282A; do
        emu_options model=$_agm conf=$(ag_h '"SCOU"')
        emu_case "$_agm SCOU not taken" --no-output -- -d $F -C P1 --time 100ms
    done
    emu_options model=U1253B conf=$(ag_h "\"VOLT +1.0000000E+00,+1.0000000E-04\"")
    emu_case "U1253B takes six decimals only" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1282A conf=$(ag_h "\"VOLT $R6\"")
    emu_case "U1282A takes eight decimals only" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1242C conf=$(ag_h "\"NCV HIGH\"")
    emu_case "U1242C NCV HIGH not taken" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1282A conf=$(ag_h "\"NCV HI\"")
    emu_case "U1282A NCV HI not taken" --no-output -- -d $F -C P1 --time 100ms
    emu_options model=U1242B conf=$(ag_h "\"VOLT:XX $R6\"")
    emu_case "U1242B VOLT with another letter" --no-output -- -d $F -C P1 --time 100ms

    # The STAT bytes of the others.
    for _ags in 100000000000000000000 010000000000000000000 000000010000000000000 110000010000000000000; do
        emu_options model=U1242B stat=$(ag_h "\"$_ags\"")
        emu_case "U1242B stat $_ags" -- -d $F -C P1 --samples 1
    done
    for _ags in 100000000000000000000 010000000000000000000 000000010000000000000 000000000001000000000; do
        emu_options model=U1242C stat=$(ag_h "\"$_ags\"")
        emu_case "U1242C stat $_ags" -- -d $F -C P1 --samples 1
    done
    for _ags in 00M000000000000000000 00V000000000000000000 010000000000000000000 000010000000000000000 000000010000000000000; do
        emu_options model=U1253B stat=$(ag_h "\"$_ags\"")
        emu_case "U1253B stat $_ags" -- -d $F -C P1 --samples 1
    done
    # Either case of the letter (protocol.c:358).
    emu_options model=U1253B stat=$(ag_h '"00m000000000000000000"')
    emu_case "U1253B stat dBm in lower case" -- -d $F -C P1 --samples 1
    emu_options model=U1253B stat=$(ag_h '"00v000000000000000000"')
    emu_case "U1253B stat dBV in lower case" -- -d $F -C P1 --samples 1
    # The guide says 1 for peak hold on a U128x, and C looks for a 4
    # (protocol.c:407). Both are here, as C has it.
    for _ags in 100000000000000000000 010000000000000000000 00M000000000000000000 00V000000000000000000 000010000000000000000 000040000000000000000 000000010000000000000 000000000001000000000; do
        emu_options model=U1282A stat=$(ag_h "\"$_ags\"")
        emu_case "U1282A stat $_ags" -- -d $F -C P1 --samples 1
    done
    # dBm and dBV reach CONF a second on, and the first turn has V's digits.
    emu_options model=U1253B stat=$(ag_h '"00M000000000000000000"')
    emu_case "U1253B dBm" -- -d $F -C P1 -c samplerate=1 --samples 2
    emu_options model=U1253B stat=$(ag_list '"00V000000000000000000"' '"000000000000000000000"')
    emu_case "U1253B dBV then not" -- -d $F -C P1 -c samplerate=1 --samples 3
    emu_options model=U1282A stat=$(ag_h '"00M000000000000000000"') fetc=$(ag_h '+1.23456789E+02')
    emu_case "U1282A dBm" -- -d $F -C P1 -c samplerate=1 --samples 2
    # The first display's alone (protocol.c:684).
    emu_options model=U1253B conf2=$(ag_h "\"VOLT:AC $R6\"") stat=$(ag_h '"00M000000000000000000"')
    emu_case "U1253B dBm leaves P2 in volts" -- -d $F -C P1,P2 -c samplerate=1 --samples 4
    # A value's own exponent cuts the digits of dB and percent, and they
    # stay cut until the next CONF.
    emu_options model=U1282A conf=$(ag_h "\"CPER:4-20mA $R8\"") fetc=$(ag_list '+1.23456789E+00' '+1.23456789E+02' '+1.23456789E+00')
    emu_case "U1282A percent digits" -- -d $F -C P1 --samples 3
    emu_options model=U1242C conf=$(ag_h "\"VOLT:HRAT $R8\"") fetc=$(ag_list '+1.23456789E+01' '+1.23456789E-02')
    emu_case "U1242C harmonic ratio digits" -- -d $F -C P1 --samples 2
    emu_options model=U1242B conf=$(ag_h "\"CPER:4-20mA $R6\"") fetc=$(ag_h '+1.23456789E+02')
    emu_case "U1242B percent digits stay" -- -d $F -C P1 --samples 1

    # FETC's values.
    emu_options fetc=$(ag_list '+9.90000000E+37' '-9.90000000E+37' '+9.90000000E+36' '-1.23475000E-03' '+0.00000000E+00')
    emu_case "values and overloads" -- -d $F -C P1 -c samplerate=20 --samples 5
    emu_case "values and overloads csv with displays off" --oracle-differs "$CSV_BUG" -- -d $F -C P1 -c samplerate=20 --samples 5 -O csv:header=false
    emu_options model=U1232A fetc=$(ag_list '+9.90000000E+37' '-9.90000000E+37' '+9.90000000E+36' '-1.23475000E-03' '+0.00000000E+00')
    emu_case "values and overloads csv" -- -d $F -c samplerate=20 --samples 5 -O csv:header=false
    emu_options model=U1272A conf=$(ag_h '"MV,1,DC"') fetc=$(ag_list '+1.23475000E+02' '+9.90000000E+37')
    emu_case "U1272A millivolts and overload" -- -d $F -C P1 --samples 2
    emu_options fetc=$(ag_list '+1.00000000E+39' '-1.00000000E+39' '+1.00000000E-50')
    emu_case "values beyond single precision" -- -d $F -C P1 --samples 3
    # None of these is a value: 300 ms, and the next.
    emu_options fetc=$(ag_list '1.23475000E+00' '+1.2347500E+00' '+1.23475000E+0' '+1.23475000E+000' '+1.23475000e+00' ' +1.23475000E+00' '+1.23475000E+00 ' '+1.23475000E+00')
    emu_case "lines that are no value" --filter "$L" --timeout 15 -- -d $F -C P1 --samples 1
    emu_options fetc=$(emu_hex '\r\n'),$(ag_h '+1.23475000E+00')
    emu_case "an empty line" -- -d $F -C P1 --samples 1
    emu_options fetc=$(emu_hex '+1.23475000E+00\000junk\r\n')
    emu_case "a line cut at a NUL" -- -d $F -C P1 --samples 1
    emu_options fetc=$(emu_hex '+1.23475000E+00\r\r\r\n')
    emu_case "a line with more CRs" -- -d $F -C P1 --samples 1
    emu_options fetc=$(emu_hex '+1.23475000E+00\n')
    emu_case "a line that ends in LF" -- -d $F -C P1 --samples 1
    # Two values with a CR between are one line, unknown: 300 ms, and the next.
    emu_options fetc=$(emu_hex '+1.23475000E+00\r+2.50000000E+00\r\n'),$(ag_h '+3.50000000E+00')
    emu_case "CR is no line end" --timeout 15 -- -d $F -C P1 --samples 1

    # Channels, in the order the driver goes through them. The first CONF of
    # a U1253B's temperature display never goes out.
    emu_options
    emu_case "U1253B P1" -- -d $F -C P1 --samples 2
    emu_case "U1253B P2" -- -d $F -C P2 --samples 2
    emu_case "U1253B P3" -- -d $F -C P3 --samples 2
    emu_case "U1253B P1 P2" -- -d $F -C P1,P2 --samples 3
    emu_case "U1253B P1 P3" -- -d $F -C P1,P3 --samples 3
    emu_case "U1253B P2 P3" -- -d $F -C P2,P3 --samples 3
    emu_case "U1253B a sample limit in a turn of displays" -- -d $F --samples 2
    emu_case "U1253B a channel renamed" -- -d $F -C P1=volts,P3=air --samples 2
    emu_case "U1253B the second turn" -- -d $F --samples 6
    emu_case "U1253B csv" -- -d $F --samples 3 -O csv:header=false
    emu_options model=U1242C
    emu_case "U1242C P1" -- -d $F -C P1 --samples 2
    emu_case "U1242C P2" -- -d $F -C P2 --samples 2
    emu_case "U1242C P1 P2" -- -d $F --samples 4
    emu_options model=U1232A
    emu_case "U1232A P1" -- -d $F --samples 2
    # A TEMP with no parameter on the second display is a copy of the third,
    # and is dropped. On a two display meter its first display is that.
    emu_options model=U1282A conf2=$(ag_h '"TEMP"')
    emu_case "U1282A TEMP on P2 dropped" -- -d $F --samples 2
    emu_case "U1282A TEMP on P2 alone" --no-output -- -d $F -C P2 --time 100ms
    emu_options model=U1282A conf=$(ag_h '"TEMP"')
    emu_case "U1282A TEMP on P1 dropped" -- -d $F --samples 2
    emu_options model=U1242C conf=$(ag_h '"TEMP"')
    emu_case "U1242C TEMP on P1 kept" -- -d $F --samples 2

    # The square wave position (protocol.c:804-809). No FETC with P1 alone.
    emu_options model=U1282A conf=$(ag_h '"SQU"')
    emu_case "U1282A SQU" --no-output -- -d $F -C P1 --time 300ms
    emu_case "U1282A SQU P3" --no-output -- -d $F -C P1,P3 --time 300ms
    # With the second display on, its CONF answer clears the mode
    # (protocol.c:669), and FETC goes out and gets the error, again and again
    # with no pause, the count of them being wall clock.
    # One FETC and one error are kept of the run, and the rest go.
    emu_case "U1282A SQU with P2 asks anyway" --no-output --tx-filter '/^bulk_out 02 464554433f0a0d0a -> 8$/{
x
/F/{
x
d
}
s/$/F/
x
}
/^bulk_in 82 4 -> 2a450d0a$/{
x
/E/{
x
d
}
s/$/E/
x
}' -- -d $F -C P1,P2 --time 500ms
    emu_options model=U1253B conf=$(ag_h '"SQU"')
    emu_case "U1253B SQU not taken" --no-output -- -d $F -C P1 --time 100ms
    # A value, the dial on SQU for a second with no FETC, and back to a value.
    emu_options model=U1282A conf=$(ag_list "\"VOLT:AC $R8\"" '"SQU"')
    emu_case "U1282A dial to SQU" -- -d $F -C P1 -c samplerate=1 --samples 2

    # The dial. `*N` starts the jobs over and forgets the quantity.
    emu_options fetc=$(ag_list '*3' '+1.23475000E+00')
    emu_case "switch" -- -d $F -C P1 --samples 1
    emu_options fetc2=$(ag_list '*3' '+1.23475000E+00')
    emu_case "switch at P2" -- -d $F --samples 3
    # The value behind the switch is for a second display it just forgot
    # (protocol.c:967-969), and goes.
    emu_options fetc2=$(emu_hex '*3\r\n+1.23475000E+00\r\n'),$(ag_h '+2.50000000E+00')
    emu_case "a value behind a switch at P2" --timeout 15 -- -d $F --samples 4
    # A switch at CONF: CONF again, not STAT (protocol.c:964).
    emu_options conf=$(ag_list '*3' "\"VOLT:AC $R6\"")
    emu_case "switch at CONF" -- -d $F -C P1 --samples 1
    emu_options model=U1242C fetc2=$(ag_list '*3' '+1.23475000E+00')
    emu_case "U1242C switch at P2" -- -d $F --samples 3
    emu_options model=U1232A fetc=$(ag_list '*3' '+1.23475000E+00')
    emu_case "U1232A switch" -- -d $F --samples 1
    emu_options model=U1272A fetc=$(ag_list '*10' '+1.23475000E+00')
    emu_case "U1272A switch of two digits" -- -d $F -C P1 --samples 1
    emu_options fetc=$(ag_list '*10' '+1.23475000E+00')
    emu_case "U1253B no switch of two digits" --timeout 15 -- -d $F -C P1 --samples 1
    emu_options fetc=$(ag_list '*B' '+1.23475000E+00')
    emu_case "low battery prompt" --timeout 15 -- -d $F -C P1 --samples 1
    emu_options fetc=$(emu_hex '*3\r\n+1.23475000E+00\r\n'),$(ag_h '+2.50000000E+00')
    emu_case "a value behind a switch" --timeout 15 -- -d $F -C P1 --samples 1

    # `*E` in a live capture asks again, where a table has it.
    emu_options model=U1282A fetc=$(ag_list '*E' '+1.23475000E+00')
    emu_case "U1282A error then a value" -- -d $F -C P1 --samples 1
    emu_options model=U1242C fetc=$(ag_list '*E' '+1.23475000E+00')
    emu_case "U1242C error then a value" -- -d $F -C P1 --samples 1
    emu_options model=U1282A conf=$(ag_list '*E' "\"VOLT:AC $R8\"")
    emu_case "U1282A error at CONF" -- -d $F -C P1 --samples 1
    emu_options fetc=$(ag_list '*E' '+1.23475000E+00')
    emu_case "U1253B no error in the table" --timeout 15 -- -d $F -C P1 --samples 1

    # The log. All sources, an index each, until a limit or an `*E`.
    for _ags in Hand Trig Auto Export; do
        emu_options model=U1282A
        emu_case "U1282A log $_ags" -- -d $F -C P1 -c data_source=Log-$_ags --samples 2
    done
    emu_options model=U1241C
    emu_case "U1241C log" -- -d $F -c data_source=Log-Hand --samples 2
    emu_options model=U1282A
    emu_case "log ends at the error" -- -d $F -c data_source=Log-Hand --samples 100
    emu_case "log ends at the error with no limit" --libsigrok --oxrok -- -d $F -c data_source=Log-Hand --continuous
    emu_options model=U1282A log=none
    emu_case "log empty" --no-output -- -d $F -c data_source=Log-Auto --samples 5
    emu_options model=U1282A
    emu_case "log channels off but P1" -- -d $F -C P1 -c data_source=Log-Hand --samples 1
    emu_case "log refused without P1" --stderr --expect-fail --filter "$REFUSED" -- -d $F -C P2 -c data_source=Log-Hand --samples 1
    emu_options model=U1253B
    emu_case "log refused on a U1253B" --stderr --expect-fail --filter "$REFUSED" -- -d $F -c data_source=Log-Hand --samples 1
    emu_options model=U1232A
    emu_case "log refused on a U1232A" --stderr --expect-fail --filter "$REFUSED" -- -d $F -c data_source=Log-Export --samples 1
    # A log entry's function, with what each table gives it.
    ag_logs U1282A 00123450000000 01123450000000 02123450000000 03123450000000 04123450000000 05123450000000 06123450000000 07123450000000 08123450000000 09123450000000 10123450000000 11123450000000 12123450000000 13123450000000
    ag_logs U1241C 00123450000000 01123450000000 02123450000000 03123450000000 04123450000000 05123450000000 06123450000000 07123450000000 08123450000000 09123450000000 10123450000000
    # Its flags, the wiki's entries first.
    ag_logs U1282A 04235201470002 00123452000000 00123450100000 00123450200000 00123450300000 00123450400000 00123450030000 00123450090000 00123450001000 04123450001000 11123450001000 06123450001000 00123450000400 00123450000010 00123450000020 00123450000040 00123459999999
    ag_logs U1241C 01000241100100 06123450001000 04123450001000 00123459999999
    # A function past the table: the same index again in 300 ms.
    emu_options model=U1282A log=$(ag_h '"14123450000000"')
    emu_case "U1282A log function past the table" --no-output -- -d $F -c data_source=Log-Hand --time 500ms
    emu_options model=U1241C log=$(ag_h '"11123450000000"')
    emu_case "U1241C log function past the table" --no-output -- -d $F -c data_source=Log-Hand --time 500ms
    # An entry of another length is no entry; on the U125x it is a STAT.
    emu_options model=U1282A log=$(ag_h '"0123450000000"')
    emu_case "log entry too short" --no-output -- -d $F -c data_source=Log-Hand --time 100ms
    emu_options model=U1282A log=$(ag_h '"001234500000000"')
    emu_case "log entry too long" --no-output -- -d $F -c data_source=Log-Hand --time 100ms
    emu_options model=U1282A log=none
    # A log entry met in a live capture counts as a sample of it.
    emu_options model=U1282A fetc=$(ag_h '"00123450000000"')
    emu_case "log entry in a live capture" -- -d $F -C P1 --samples 1

    # Rates: the time between FETCs, and CONF and STAT each second ahead of the
    # FETC due with them.
    emu_options
    emu_case "samplerate 1" -- -d $F -C P1 -c samplerate=1 --samples 2
    emu_case "samplerate 2" -- -d $F -C P1 -c samplerate=2 --samples 3
    emu_case "samplerate 3" -- -d $F -C P1 -c samplerate=3 --samples 2
    emu_case "samplerate 10" -- -d $F -C P1 -c samplerate=10 --samples 3
    emu_case "samplerate 20" -- -d $F -C P1 -c samplerate=20 --samples 3
    # Before the second CONF only: the 5 FETCs per CONF at this rate are a margin,
    # and a unit test holds that order on a clock of its own.
    emu_case "samplerate default" -- -d $F -C P1 --samples 3
    emu_case "limit in -c" --libsigrok --oxrok -- -d $F -C P1 -c limit_samples=2 --continuous
    emu_case "time limit" -- -d $F -C P1 -c samplerate=1 --time 500ms
    emu_case "limit_time in -c" --libsigrok --oxrok -- -d $F -C P1 -c samplerate=1 -c limit_time=500 --continuous

    # Silence: a job is given up after 300 ms. CONF and STAT are in their
    # second then, so the next runs a pass later. FETC's interval is over, so it
    # goes again at once and for good, with no more CONF (protocol.c:112-116).
    emu_options answers=0
    # The FETCs run into one transfer's line, and their number is wall clock.
    emu_case "silent" --no-output --tx-filter 's/^bulk_out 02 434f4e463f0a0d0a535441543f0a0d0a(464554433f0a0d0a)+ -> [0-9]+$/CONF, STAT, and FETC again and again/' -- -d $F -C P1 --time 1500ms
    emu_options answers=3
    emu_case "stops answering" --tx-filter 's/^bulk_out 02 (464554433f0a0d0a)+ -> [0-9]+$/FETC again and again/' -- -d $F -C P1 --time 1500ms

    emu_options answers=0
    emu_case "lib silent waits on" --libsigrok --hangs --timeout 3 --no-output -- -d $F -C P1 --samples 1

    # A line that fills the buffer: 254 bytes and a LF fit, 255 stop it reading.
    emu_options fetc=$(ag_h "$(printf 'x%.0s' $(seq 1 253))"),$(ag_h '+1.23475000E+00')
    emu_case "a line of 255 bytes" --timeout 15 -- -d $F -C P1 --samples 1
    emu_options fetc=$(ag_h "$(printf 'x%.0s' $(seq 1 254))"),$(ag_h '+1.23475000E+00')
    emu_case "a line of 256 bytes" --oracle-bug "$WEDGED_BUG" --oracle-hangs --timeout 3 -- -d $F -C P1 --samples 1
    # A log entry that never is one, asked for ever.
    emu_options model=U1282A log=$(ag_h '"14123450000000"')
    emu_case "lib log entry that never is one" --libsigrok --hangs --timeout 3 --no-output -- -d $F -c data_source=Log-Hand --samples 1
    # A NUL first leaves g_strsplit an empty string, an empty vector, and
    # api.c:146 hands strcmp its NULL.
    emu_options id=$(emu_hex '\000\r\n')
    emu_case "id only a NUL" --oracle-bug "$NUL_BUG" --keep-header --no-output -- -d $F --scan

    # The meter stops reading. The tty holds 16 writes; after those each gets
    # 1 ms and "Failed to send." (protocol.c:226-228), which leaves the job
    # running, so one write per 300 ms.
    emu_options out-hold=1
    emu_tty_closing_wait 100
    emu_case "stops reading" --stderr --timeout 20 --filter "$L
/^sr: agilent-dmm: Failed to send\\.$/{
x
/F/{
x
d
}
s/$/F/
x
}" -- -d $F -C P1 --time 8000ms
    emu_tty_closing_wait 3000
    emu_options answers=3
    # C's two error lines repeat in turn, as often as time allows, so this one
    # is quiet, and the next holds them. A unit test holds oxrok's.
    emu_case "unplug in a capture" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $F -C P1 --samples 3 -l 0
    emu_case "lib unplug in a capture" --libsigrok --hangs --unplug model --timeout 3 --stderr --filter '3,$d' --tx-filter '/^unplug$/q' -- -d $F -C P1 --samples 3
    emu_options answers=4
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $F -C P1 --samples 9
    # oxrok's bound on silence, and C's wait, of which "lib silent" has the wire.
    emu_options answers=0
    emu_case "silent with only a sample limit" --oracle-bug "$SILENT_DIFFERS" --oracle-hangs --timeout 13 --stderr --filter "$L" --tx-filter 's/^bulk_out 02 434f4e463f0a0d0a535441543f0a0d0a(464554433f0a0d0a)+ -> [0-9]+$/CONF, STAT, and FETC until the bound/' -- -d $F -C P1 --samples 1
    emu_options id=none
    emu_case "unplug in the scan" --unplug model --timeout 15 --keep-header --no-output --tx-filter '/^unplug$/q' -- -d $F --scan

    # The USB side, the shell's options.
    emu_options chunk=1
    emu_case "one byte per transfer" -- -d $F -C P1 --samples 2
    emu_options stall=line
    emu_case "scan line coding stalled" --keep-header -- -d $F --scan
    emu_options stall=lines
    emu_case "scan control lines stalled" --keep-header --tx-filter '/^ctl_out 21 20 /d' -- -d $F --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    F=agilent-dmm:conn=$EMU_TTY
}
