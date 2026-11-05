# A Fluke meter behind its IR cable (emu/src/models/fluke-dmm.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=fluke-dmm
EMU_CDC=1
EMU_DRIVER=fluke-dmm
EMU_GCOV=fluke-dmm
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# The ID answer TEXT on a cable with no line to get wrong, so that every
# rate reaches the meter.
fl_id() {
    emu_options uart=native id=$(emu_hex "$1")
}

# A 187 whose display shows each of the arguments in turn.
fl_display() {
    _l=
    for _t in "$@"; do
        _l=$_l${_l:+,}$(emu_hex "0\rQM,$_t\r")
    done
    emu_options model=187 qm=$_l
}

# A 287 that gives each of the arguments in turn.
fl_fields() {
    _l=
    for _t in "$@"; do
        _l=$_l${_l:+,}$(emu_hex "0\r$_t\r")
    done
    emu_options model=287 qm=$_l
}

# A 199B with this answer to QM and this one to QM N.
fl_scope() {
    emu_options model=199B qm=$(emu_hex "0\r$1\r") data=$(emu_hex "0\r$2\r")
}

# TEXT N times.
fl_many() {
    printf "$1%.0s" $(seq 1 "$2")
}

emu_cases() {
    emu_conns
    # A capture on wall time: its number of QMs and answers varies. The
    # sed names exactly what may go, so that anything else still shows.
    QM_287='/^bulk_out 02 (514d0d)+ -> [0-9]+$/d; /^bulk_in 82 [0-9]+ -> (300d312e30333237453[0-9a-f]*0d)+$/d'
    FIRST='2,$d'

    # A scan tries 115200, 9600, 1200, three IDs each, until one answers.
    for model in 87 89 187 189 199B 287 289; do
        emu_options model=$model
        emu_case "scan $model" --keep-header -- -d $F --scan
    done
    emu_options model=287
    emu_case "show" --keep-header -- -d $F --show
    emu_options model=199B
    emu_case "show 199B" --keep-header -- -d $F --show
    emu_options
    emu_case "get conn" --keep-header --stderr --expect-fail -- -d $F --get conn
    emu_case "get limit_samples" --keep-header --stderr --expect-fail -- -d $F --get limit_samples
    emu_case "no such node" --keep-header --no-output -- -d fluke-dmm:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d fluke-dmm --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d fluke-dmm --show

    # serialcomm= is the only line tried.
    emu_case "serialcomm as the meter" --keep-header -- -d $F:serialcomm=115200/8n1 --scan
    emu_case "serialcomm misset" --keep-header --no-output -- -d $F:serialcomm=9600/8n1 --scan
    emu_options model=199B
    emu_case "serialcomm as the 199B" --keep-header -- -d $F:serialcomm=1200/8n1 --scan
    emu_options
    # sigrok-cli opens the port before it reads the parameters: the open's
    # lines are all that may differ.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $F:serialcomm=fast --scan

    # What a scan takes for an ID (fluke_scan, api.c:85-140). A meter that
    # never matches is asked nine times, three at each rate.
    fl_id '0\rFLUKE 287,V1.16,1\r'
    emu_case "id short serial" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 287,   V1.16,1\r'
    emu_case "id spaces before the version" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 287,V1.16,1,2,3\r'
    emu_case "id more fields" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 287,,\r'
    emu_case "id empty fields" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 287,V1.16\r'
    emu_case "id no third field" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 287\r'
    emu_case "id no fields" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 87\r'
    emu_case "id too short" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 87,,\r'
    emu_case "id just long enough" --keep-header -- -d $F --scan
    fl_id '0\rFLUKEX287,V1.16,1\r'
    emu_case "id any byte after FLUKE" --keep-header -- -d $F --scan
    fl_id '0\rfluke 287,V1.16,1\r'
    emu_case "id in lower case" --keep-header --no-output -- -d $F --scan
    fl_id '0\rACME 287,V1.16,1234\r'
    emu_case "id another maker" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 199B;V11.40;2008-06-11;ENGLISH,FRENCH,GERMAN\r'
    emu_case "id semicolons and a comma after them" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 199B;V11.40,2008-06-11;ENGLISH\r'
    emu_case "id a comma at 17" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 199B;V1,2,3;ENGLISH\r'
    emu_case "id a comma before 15" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 199B;V11.40\r'
    emu_case "id semicolons no third field" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 287;V1.16;1\r'
    emu_case "id handheld with semicolons" --keep-header -- -d $F --scan
    fl_id '1\r'
    emu_case "id refused" --keep-header --no-output -- -d $F --scan
    fl_id '2\rFLUKE 287,V1.16,1\r'
    emu_case "id refused and answered" --keep-header --no-output -- -d $F --scan
    fl_id '00\rFLUKE 287,V1.16,1\r'
    emu_case "id ack too long" --keep-header --no-output -- -d $F --scan
    fl_id 'FLUKE 287,V1.16,1\r'
    emu_case "id no ack" --keep-header --no-output -- -d $F --scan
    fl_id '0\r'
    emu_case "id ack only" --keep-header --no-output -- -d $F --scan
    fl_id '0\nFLUKE 287,V1.16,1\n'
    emu_case "id ends in LF" --keep-header -- -d $F --scan
    fl_id '0\r\nFLUKE 287,V1.16,1\r\n'
    emu_case "id ends in CR LF" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 287,V1.16,1'
    emu_case "id no last CR" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 287,V1.16,\0001\r'
    emu_case "id cut at a NUL" --keep-header -- -d $F --scan
    fl_id '0\rFLUKE 287,V1\0.16,1\r'
    emu_case "id NUL in the version" --keep-header --no-output -- -d $F --scan
    fl_id '0'
    emu_case "id ack without its CR" --keep-header --no-output -- -d $F --scan
    fl_id "0\rFLUKE 287,V1.16,$(fl_many x 200)\r"
    emu_case "id longer than the buffer" --keep-header -- -d $F --scan
    fl_id "0\rFLUKE 287,$(fl_many v 200),1\r"
    emu_case "id version past the buffer" --keep-header --no-output -- -d $F --scan
    # One bad answer, then a good one (api.c:83-84's idle state).
    emu_options id=$(emu_hex '1\r'),$(emu_hex '0\rFLUKE 287,V1.16,1\r')
    emu_case "id good at the second try" --keep-header -- -d $F --scan
    emu_options id=$(emu_hex '1\r'),$(emu_hex '1\r'),$(emu_hex '0\rFLUKE 287,V1.16,1\r')
    emu_case "id good at the third try" --keep-header -- -d $F --scan
    emu_options uart=native id=$(emu_hex '1\r'),$(emu_hex '1\r'),$(emu_hex '1\r'),$(emu_hex '0\rFLUKE 287,V1.16,1\r')
    emu_case "id good at the next rate" --keep-header -- -d $F --scan
    emu_options id=none
    emu_case "id silence" --keep-header --no-output -- -d $F --scan
    # A model the table lacks (api.c:115, a table with no end marker).
    fl_id '0\rFLUKE 999,V1.16,1\r'
    emu_case "id unknown model" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE,V1.16,1234\r'
    emu_case "id no model" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 2870,V1.16,1\r'
    emu_case "id model with more" --keep-header --no-output -- -d $F --scan
    fl_id '0\rFLUKE 28,V1.16,1234\r'
    emu_case "id model with less" --keep-header --no-output -- -d $F --scan

    # The 8x and 18x: the display as text (fluke-18x.c).
    emu_options model=187
    emu_case "187 samples 1" -- -d $F --samples 1
    emu_case "187 samples 3" -- -d $F --samples 3
    emu_case "187 csv" -- -d $F --samples 2 -O csv:header=false
    emu_options model=87
    emu_case "87 samples 1" -- -d $F --samples 1
    emu_options model=89
    emu_case "89 samples 1" -- -d $F --samples 1
    emu_options model=189
    emu_case "189 samples 1" -- -d $F --samples 1
    fl_display '+1.234 V DC' '-1.234 mV DC' '+1.234 V AC' '+1.234 mV AC' '+1.234 V AC+DC' '+1.234 mV AC+DC' '+12 V DC' '+.5 V DC' '+5. V DC' '1E2 V DC' '1.5E-1 V DC'
    emu_case "187 volts" -- -d $F --samples 11
    fl_display '+1.234 A DC' '+1.234 mA DC' '+1.234 uA DC' '+1.234 A AC' '+1.234 mA AC' '+1.234 uA AC' '+1.234 A AC+DC' '+1.234 mA AC+DC' '+1.234 uA AC+DC'
    emu_case "187 amperes" -- -d $F --samples 9
    fl_display '+1.234 Ohms' '+1.234 kOhms' '+1.234 MOhms' '+1.234 nS' '+1.234 Farads' '+1.234 mFarads' '+1.234 uFarads' '+1.234 nFarads'
    emu_case "187 resistance conductance capacitance" -- -d $F --samples 8
    fl_display '+1.234 dBV' '+1.234 dBm' '+21.5 Deg C' '+70.7 Deg F' '+1.234 Hz' '+1.234 kHz' '+50.0 %%' '+1.234 ms'
    emu_case "187 the others" -- -d $F --samples 8
    # Out of range: the text the driver looks for, the unit after a dot.
    fl_display 'Out of range. V DC' 'Out of range. mV AC' 'Out of range. dBm' 'Out of range. Ohms' 'Out of range. kOhms' 'Out of range. Farads' 'Out of range. mFarads' 'Out of range. Deg C' 'Out of range. A DC' 'Out of range. mA AC+DC' 'Out of range. Hz' 'Out of range. ms'
    emu_case "187 out of range" -- -d $F --samples 12
    # No sample from these, each is an answer: the next line ends the case.
    fl_display 'Out of range. nS' '+1.5 V DC'
    emu_case "187 out of range nS" -- -d $F --samples 1
    fl_display 'Out of range. %%' '+1.5 V DC'
    emu_case "187 out of range percent" -- -d $F --samples 1
    fl_display 'Out of range V DC' '+1.5 V DC'
    emu_case "187 out of range no dot" -- -d $F --samples 1
    # As the copy of Fluke's specification spells them.
    fl_display '+47.66 KOhms' '+1.5 V DC'
    emu_case "187 the specification's KOhms" -- -d $F --samples 2
    fl_display '-121.43 VDC' '+1.5 V DC'
    emu_case "187 the specification's VDC" -- -d $F --samples 1
    fl_display 'Out of Range mVDC' '+1.5 V DC'
    emu_case "187 the specification's out of range" -- -d $F --samples 1
    fl_display '+1.234 mS' '+1.5 V DC'
    emu_case "187 the specification's mS" -- -d $F --samples 1
    fl_display '+1.234 parsecs' '+1.5 V DC'
    emu_case "187 an unknown unit" -- -d $F --samples 1
    fl_display 'x.yz V DC' '+1.5 V DC'
    emu_case "187 not a float" -- -d $F --samples 1
    fl_display '+1.2.3 V DC' '+1E999 V DC' '0x10 V DC' 'inf V DC' 'nan V DC' '+1.5 V DC'
    emu_case "187 floats refused" -- -d $F --samples 1
    fl_display '+1E-400 V DC' '+1.5 V DC'
    emu_case "187 float underflow" -- -d $F --samples 1
    fl_display '+1E39 V DC' '+1.5E40 mV AC' '-1E39 Ohms'
    emu_case "187 float beyond single precision" -- -d $F --samples 3
    fl_display ' +1.5 V DC' '+1.5 V DC'
    emu_case "187 a space first" -- -d $F --samples 1
    fl_display '+1.5   V DC' '+1.5 mVx DC' '+1.5 xV DC' '+1.5 V DC,more'
    emu_case "187 odd spacing and prefixes" -- -d $F --samples 3
    fl_display '+1.5' '+2.5 V DC' '+1.5' '+1.5'
    emu_case "187 no unit" -- -d $F --samples 2
    emu_options model=187 qm=$(emu_hex '0\rQM\r0\rQM,+1.5 V DC\r')
    emu_case "187 no reading then one" -- -d $F --samples 1
    emu_options model=187 qm=$(emu_hex '0\rXX,+1.5 V DC\r0\rQM,+2.5 V DC\r')
    emu_case "187 another command's name" -- -d $F --samples 1
    emu_options model=187 qm=$(emu_hex '0\r\r0\rQM,+2.5 V DC\r')
    emu_case "187 an empty line" -- -d $F --samples 1
    emu_options model=187 qm=$(emu_hex '1\r0\rQM,+2.5 V DC\r')
    emu_case "187 refused then answered" -- -d $F --samples 1
    emu_options model=187 qm=$(emu_hex '5\r')
    emu_case "187 refused" --no-output --tx-filter '/^bulk_out 02 (514d0d)+ -> [0-9]+$/d; /^bulk_in 82 [0-9]+ -> (350d)+$/d' -- -d $F --time 500ms
    emu_options model=187 qm=$(emu_hex '0\r\0003\rQM,+2.5 V DC\r')
    emu_case "187 a NUL line" -- -d $F --samples 1

    # The 287 and 289: value, unit, state, attribute (fluke-28x.c).
    emu_options
    emu_case "287 samples 1" -- -d $F --samples 1
    emu_case "287 samples 3" -- -d $F --samples 3
    emu_case "287 csv" -- -d $F --samples 2 -O csv:header=false
    emu_case "287 limit in -c" --libsigrok -- -d $F -c limit_samples=2 --continuous
    emu_case "287 a channel named" -- -d $F -C P1=volts --samples 1
    emu_options model=289
    emu_case "289 samples 1" -- -d $F --samples 1
    fl_fields '1.5E0,VDC,NORMAL,NONE' '1.5E0,VAC,NORMAL,NONE' '1.5E0,ADC,NORMAL,NONE' '1.5E0,AAC,NORMAL,NONE' '1.5E0,VAC_PLUS_DC,NORMAL,NONE' '1.5E0,AAC_PLUS_DC,NORMAL,NONE' '1.5E0,V,NORMAL,NONE' '1.5E0,A,NORMAL,NONE' '1.5E3,OHM,NORMAL,NONE' '1.5E-6,SIE,NORMAL,NONE' '1.5E3,Hz,NORMAL,NONE' '1.5E-3,S,NORMAL,NONE' '1.5E-6,F,NORMAL,NONE' '21.5E0,CEL,NORMAL,NONE' '70.7E0,FAR,NORMAL,NONE' '50.0E0,PCT,NORMAL,NONE' '1.5E0,dBm,NORMAL,NONE' '1.5E0,dBV,NORMAL,NONE'
    emu_case "287 every unit" -- -d $F --samples 18
    emu_case "287 every unit csv" -- -d $F --samples 18 -O csv:header=false
    fl_fields '+9.99999999E+37,VDC,OL,NONE' '-9.99999999E+37,VDC,OL_MINUS,NONE' '0E0,CEL,OPEN_TC,NONE' '+9.99999999E+37,OHM,OL,NONE'
    emu_case "287 the states that give a sample" -- -d $F --samples 4
    fl_fields '1.5E0,VDC,INVALID,NONE' '1.5E0,VDC,BLANK,NONE' '1.5E0,VDC,DISCHARGE,NONE' '1.5E0,VDC,SOMETHING,NONE' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 the states that give none" --stderr --filter '/^sr: /!d' -- -d $F --samples 1 -l 2
    fl_fields '1.5E0,OHM,NORMAL,OPEN_CIRCUIT' '1.5E0,OHM,NORMAL,SHORT_CIRCUIT' '1.5E0,OHM,OL,OPEN_CIRCUIT' '1.5E0,OHM,BLANK,SHORT_CIRCUIT' '1.5E0,VDC,NORMAL,OPEN_CIRCUIT' '1.5E0,VDC,NORMAL,GLITCH_CIRCUIT' '1.5E0,ADC,NORMAL,HIGH_CURRENT' '1.5E0,VDC,NORMAL,SOMETHING'
    emu_case "287 attributes" -- -d $F --samples 7
    fl_fields '1.5E0,OHM,NORMAL,GLITCH_CIRCUIT' '1.5E0,OHM,NORMAL,GOOD_DIODE' '1.5E0,OHM,NORMAL,LO_OHMS' '1.5E0,OHM,NORMAL,NEGATIVE_EDGE' '1.5E0,OHM,NORMAL,POSITIVE_EDGE' '1.5E0,OHM,NORMAL,HIGH_CURRENT' '1.5E0,OHM,NORMAL,SOMETHING' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 attributes that leave resistance no sample" --stderr --filter '/^sr: /!d' -- -d $F --samples 1 -l 2
    fl_fields 'x,VDC,NORMAL,NONE' '1.5E0,VOLTS,NORMAL,NONE' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 bad value and unit" --stderr --filter '/^sr: /!d' -- -d $F --samples 1 -l 2
    fl_fields '1.5' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 value alone" --stderr --filter '/^sr: /!d' -- -d $F --samples 1 -l 2
    fl_fields '1.5E0,VDC' '1.5E0,VDC,NORMAL,NONE'
    # A segmentation fault: tokens[3] is past the array (fluke-28x.c:196).
    emu_case "287 no state" --expect-fail --no-output -- -d $F --samples 1
    fl_fields '1.5E0,VDC,NORMAL' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 no attribute" -- -d $F --samples 1
    fl_fields '1.5E0,OHM,NORMAL' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 resistance no attribute" -- -d $F --samples 1
    fl_fields '1.5E0,VDC,NORMAL,NONE,more,fields'
    emu_case "287 more fields" -- -d $F --samples 1
    fl_fields '-0.0E0,VDC,NORMAL,NONE' '0E-5,VDC,NORMAL,NONE' '1234.5678E-3,VDC,NORMAL,NONE' '1E2,VDC,NORMAL,NONE'
    emu_case "287 values" -- -d $F --samples 4
    fl_fields '0E3000000000,VDC,NORMAL,NONE' '0E99999999999999999999,VDC,NORMAL,NONE' '1.5E0,VDC,NORMAL,NONE'
    emu_case "287 exponents out of range" -- -d $F --samples 2
    # The driver's buffer: 254 bytes and a CR fit, 255 stop it reading.
    emu_options qm=$(emu_hex "0\r1.5E0,VDC,NORMAL,NONE,$(fl_many x 232)\r")
    emu_case "287 a line of 254 bytes" -- -d $F --samples 1
    emu_options qm=$(emu_hex "0\r1.5E0,VDC,NORMAL,NONE,$(fl_many x 233)\r")
    emu_case "287 a line of 255 bytes" --hangs --timeout 3 --no-output -- -d $F --samples 1

    # The 199B: what is shown, then its value (fluke-190.c). Each reply to
    # QM N asks for the next, so a capture runs as fast as the meter does,
    # and one more goes out after the last sample. The QM every second is
    # too far off to show in these.
    emu_options model=199B
    emu_case "199B samples 1" -- -d $F --samples 1
    emu_case "199B samples 3" -- -d $F --samples 3
    emu_case "199B csv" -- -d $F --samples 2 -O csv:header=false
    fl_scope 11,1,3,1,1,0,1 +1.5E+00
    emu_case "199B volts DC" -- -d $F --samples 1
    fl_scope 11,1,3,1,2,0,1 +1.5E+00
    emu_case "199B volts AC" -- -d $F --samples 1
    fl_scope 11,1,3,1,3,0,1 +1.5E+00
    emu_case "199B volts AC+DC" -- -d $F --samples 1
    fl_scope 11,1,3,1,15,0,1 +1.5E+00
    emu_case "199B diode" -- -d $F --samples 1
    fl_scope 11,1,3,1,7,0,1 +1.5E+00
    emu_case "199B volts of another type" -- -d $F --samples 1
    fl_scope 11,1,3,2,1,0,1 +1.5E-01
    emu_case "199B amperes DC" -- -d $F --samples 1
    fl_scope 11,1,3,2,2,0,1 +1.5E-01
    emu_case "199B amperes AC" -- -d $F --samples 1
    fl_scope 11,1,3,2,3,0,1 +1.5E-01
    emu_case "199B amperes AC+DC" -- -d $F --samples 1
    fl_scope 11,1,3,2,7,0,1 +1.5E-01
    emu_case "199B amperes of another type" -- -d $F --samples 1
    fl_scope 11,1,3,3,1,0,1 +1.5E+03
    emu_case "199B resistance" -- -d $F --samples 1
    fl_scope 11,1,3,3,1,0,1 9.9E+37
    emu_case "199B resistance OL" -- -d $F --samples 1
    fl_scope 11,1,3,3,16,0,1 +1.5E+01
    emu_case "199B continuity closed" -- -d $F --samples 1
    fl_scope 11,1,3,3,16,0,1 9.9E+37
    emu_case "199B continuity open" -- -d $F --samples 1
    fl_scope 11,1,3,12,1,0,1 +2.1E+01
    emu_case "199B celsius" -- -d $F --samples 1
    fl_scope 11,1,3,13,1,0,1 +7.0E+01
    emu_case "199B fahrenheit" -- -d $F --samples 1
    fl_scope 11,1,3,1,1,0,1 9.9E+37
    emu_case "199B volts OL" -- -d $F --samples 1
    fl_scope 19,1,3,1,1,1,1 -1.5E+00
    emu_case "199B relative" -- -d $F --samples 1
    fl_scope 11,1,3,3,1,0,1,19,1,3,1,1,1,1 +1.5E+00
    emu_case "199B absolute then relative" -- -d $F --samples 1
    fl_scope 19,1,3,1,1,1,1,11,1,3,3,1,0,1 +1.5E+00
    emu_case "199B relative then absolute" -- -d $F --samples 1
    fl_scope 11,1,3,1,1,0,1,19 +1.5E+00
    emu_case "199B a set cut short" -- -d $F --samples 1
    # The relative set's unit is one it lacks: QM 11, and nothing to say.
    fl_scope 11,1,3,1,1,0,1,19,1,3,4,1,1,1 +1.5E+00
    emu_case "199B relative in a unit it lacks" --no-output --tx-filter '/^bulk_out 02 (514d2031310d)+ -> [0-9]+$/d; /^bulk_in 82 [0-9]+ -> (300d2b312e35452b30300d)+$/d' -- -d $F --time 200ms
    # No QM N follows any of these: one QM, and none till a second is up.
    fl_scope 11,0,3,1,1,0,1 +1.5E+00
    emu_case "199B not valid" --no-output -- -d $F --time 200ms
    fl_scope 11,1,1,1,1,0,1 +1.5E+00
    emu_case "199B scope mode" --no-output -- -d $F --time 200ms
    fl_scope 21,1,3,1,1,0,1 +1.5E+00
    emu_case "199B reading 21" --no-output -- -d $F --time 200ms
    fl_scope 11,1,3,0,1,0,1 +1.5E+00
    emu_case "199B turned off" --no-output -- -d $F --time 200ms
    fl_scope 11,1,3,4,1,0,1 +1.5E+00
    emu_case "199B watts" --no-output -- -d $F --time 200ms
    fl_scope 11,1,3,3,7,0,1 +1.5E+00
    emu_case "199B resistance of another type" --no-output -- -d $F --time 200ms
    # Bad values: no sample, and it asks for the next at once.
    emu_options model=199B data=$(emu_hex '0\rx\r'),$(emu_hex '0\r0.000E+00\r'),$(emu_hex '0\r+1.5E+00\r')
    emu_case "199B bad value and zero" --stderr --filter '/^sr: /!d' -- -d $F --samples 1 -l 2
    emu_options model=199B data=$(emu_hex '0\r+1.5E+00,+2.5E+00,3\r')
    emu_case "199B more values on a line" -- -d $F --samples 1
    emu_options model=199B data=$(emu_hex '2\r')
    emu_case "199B reading refused" --no-output -- -d $F --time 200ms
    emu_options model=199B qm=$(emu_hex '2\r')
    emu_case "199B QM refused" --no-output -- -d $F --time 200ms
    emu_options model=199B answers=1
    emu_case "199B silent after the metadata" --no-output -- -d $F --time 200ms
    # The QM a second after the last, and the readings go on.
    emu_options model=199B
    emu_case "199B a QM each second" --filter "$FIRST" --tx-filter '/^bulk_out 02 (514d0d|514d2031310d)+ -> [0-9]+$/d; /^bulk_in 82 [0-9]+ -> (300d[0-9a-f]+0d)+$/d' -- -d $F --time 1500ms

    # Silence and pace: the next QM comes a second after the last on a meter
    # that does not answer, and 100 ms after one that does.
    emu_options answers=0
    emu_case "287 silent" --no-output -- -d $F --time 1500ms
    emu_options answers=1
    emu_case "287 stops answering" -- -d $F --time 1500ms
    emu_options
    emu_case "287 time limit" --filter "$FIRST" --tx-filter "$QM_287" -- -d $F --time 350ms
    emu_case "287 limit_time in -c" --libsigrok --filter "$FIRST" --tx-filter "$QM_287" -- -d $F -c limit_time=350 --continuous

    # The USB side, the shell's options.
    emu_options chunk=1
    emu_case "287 one byte per transfer" -- -d $F --samples 2
    emu_options stall=line
    emu_case "scan line coding stalled" --keep-header --no-output -- -d $F --scan
    emu_options stall=lines
    emu_case "scan control lines stalled" --keep-header --tx-filter '/^ctl_out 21 20 /d' -- -d $F --scan

    # An unplug during the scan and during a capture, and an interrupt.
    emu_options id=none
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --stderr --filter '/^sr: /!d' --tx-filter '/^unplug$/q' -- -d $F --scan -l 2
    emu_options answers=2
    emu_case "unplug in a capture" --hangs --unplug model --timeout 4 --stderr --filter '/^sr: /!d' -- -d $F --samples 3
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    F=fluke-dmm:conn=$EMU_TTY
}
