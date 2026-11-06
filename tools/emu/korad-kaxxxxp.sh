# A Korad KAxxxxP power supply (emu/src/models/korad-kaxxxxp.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=korad-kaxxxxp
EMU_CDC=1
EMU_DRIVER=korad-kaxxxxp
EMU_FULL=$EMU_DRIVER
EMU_GCOV=korad-kaxxxxp
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only, and what it adds after the
    # error sigrok-cli names.
    L='s/^oxrok: err: oxrok::drivers::korad_kaxxxxp: /sr: korad-kaxxxxp: /'
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./'
    NAN_BUG="api.c:432-433's two comparisons let a NaN through, and protocol.c:210 sends 'VSET1:  nan' to a live output; oxrok refuses it"
    ORDER_BUG="sigrok-cli sets -c's options in hash table order (session.c:700-701), here OUT1 before VSET1, so the output goes on at the old voltage; oxrok sets them as given, but an output going on last and one going off first"
    HUP_BUG="receive_data drops get_value's result (protocol.c:452) and sends the last values again until the limit, with no device there; oxrok ends the capture"
    PARTIAL_BUG="sigrok-cli sets -c's options one at a time in hash table order (session.c:700-707), so OUT1 goes out before a later option is refused; oxrok checks them all first and sends nothing"
    LOOSE_BUG="sigrok-cli takes what is no value (session.c's opt_to_gvar, strtod and boolstring) and sends what it makes of it; oxrok refuses it and sends nothing"
    SILENT_BUG="read_chars takes no reply for no error (protocol.c:138-139, 152) and sr_atof_ascii(\"\") is 0.0 (protocol.c:333), so a silent device reads 0 V; oxrok sends no sample and fails --get"
    ZERO_BUG="api.c:432 and 440 let -0.0 through, and protocol.c:207 and 211 send it as -0.00; oxrok sends 0"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # Every entry of api.c's models[], with the ID strings its table had
    # up to libsigrok d241cc2 (72-2705's is made from its name). A scan
    # asks *IDN?, then IOUT1? ISET1? VOUT1? VSET1? STATUS?, and sets nothing:
    # setters=0 flags a setting, which the .tx would also show.
    for id in "KORADKA3005PV2.0" "KORAD KA3005P V5.5" "KORADKD3005PV2.0" "KORAD KD3005P V6.8" \
        "KORAD KD6005P V2.2" "RND 320-KA3005P V5.5" "RND 320-KD3005P V4.2" "S-LS-31 V2.0" \
        "TENMA 72-2535 V2.1" "TENMA72-2540V2.0" "TENMA 72-2540 V5.2" "TENMA72-2550V2.0" \
        "TENMA 72-2705 V2.0" "TENMA 72-2710 V6.6" "VELLEMANLABPS3005DV2.0" "VELLEMANPS3005DV1.3" \
        "VELLEMANPS3005DV2.0"; do
        emu_options setters=0 idn=$(emu_hex "$id")
        emu_case "scan $id" --keep-header -- -d $K --scan
    done

    # How far the ID check stretches (model_matches, api.c:96-199).
    emu_options setters=0 idn=$(emu_hex 'KORADKA3005PV2.0\001')
    emu_case "scan KA3005P trailing 01" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'KORADKA3005PV2.0\274')
    emu_case "scan KA3005P trailing bc" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'KORAD KA3005P V5.8 SN:12345678')
    emu_case "scan a serial number" --keep-header -- -d $K --scan
    emu_case "show a serial number" --keep-header -- -d $K --show
    emu_options setters=0 idn=$(emu_hex 'KORAD KA3005P SN:1 SN:2')
    emu_case "scan the last SN counts" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'korad ka3005p v2.0')
    emu_case "scan in lower case" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'KORAD  KA3005P')
    emu_case "scan with spaces and no version" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'KORADKA3005PV2.0\000junk')
    emu_case "scan cut at a NUL" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'KORADKD3005PV2.0\001')
    emu_case "scan KD3005P has no trailing byte" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'RND 320-KA3005P V5.5 ')
    emu_case "scan RND with a space after the version" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'RND 320-KA3005P V5.5x')
    emu_case "scan RND with garbage" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'ACME PSU 9000')
    emu_case "scan an unknown ID" --stderr --filter "$L" --keep-header -- -d $K --scan
    # A reply longer than the buffer: 48 of it are read, no match.
    emu_options setters=0 idn=$(emu_hex 'KORADKA3005PV2.0 0123456789012345678901234567890123456789')
    emu_case "scan a reply too long" --keep-header --no-output -- -d $K --scan
    # Silence: 100 reads of 60 ms each, then no model.
    emu_options setters=0 idn=none
    emu_case "scan no answer" --keep-header --no-output -- -d $K --scan
    # force_detect stands in for what did not match, or did not answer.
    emu_case "force_detect on silence" --keep-header -- -d $K:force_detect=KORADKD3005PV2.0 --scan
    emu_options setters=0 idn=$(emu_hex 'ACME PSU 9000')
    emu_case "force_detect on an unknown ID" --keep-header -- -d "$K:force_detect=TENMA 72-2535 V2.1" --scan
    emu_case "force_detect unknown too" --keep-header --no-output -- -d $K:force_detect=NOSUCH --scan
    emu_options setters=0
    emu_case "force_detect not needed" --keep-header -- -d $K:force_detect=KORADKD3005PV2.0 --scan
    emu_options setters=0 idn=none
    emu_case "force_detect empty" --keep-header --stderr --expect-fail -- -d "$K:force_detect=" --scan
    emu_options setters=0
    emu_case "no such node" --keep-header --no-output -- -d korad-kaxxxxp:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d korad-kaxxxxp --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d korad-kaxxxxp --show

    # The line: native USB does not mind; behind the RS232 port it must fit.
    emu_case "serialcomm on the native port" --keep-header -- -d $K:serialcomm=19200/8n1 --scan
    emu_options setters=0 uart=9600/8n1
    emu_case "serialcomm as the wiki" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset behind the UART" --keep-header --no-output -- -d $K:serialcomm=19200/8n1 --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses
    # them first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan

    # What --show and --get ask for, per range in the table.
    emu_options setters=0
    emu_case "show" --keep-header -- -d $K --show
    emu_options setters=0 idn=$(emu_hex 'KORAD KD6005P V2.2') vmax=61
    emu_case "show KD6005P" --keep-header -- -d $K --show
    emu_options setters=0 idn=$(emu_hex 'TENMA 72-2535 V2.1') imax=3.1
    emu_case "show 72-2535" --keep-header -- -d $K --show
    emu_options setters=0 idn=$(emu_hex 'TENMA72-2550V2.0') vmax=61 imax=3.1
    emu_case "show 72-2550" --keep-header -- -d $K --show
    emu_options setters=0 output=1 ocp=1 ovp=1 load=10
    emu_case "show on, with a load" --keep-header -- -d $K --show
    for key in conn voltage voltage_target current current_limit enabled regulation \
        ovp_enabled ocp_enabled limit_samples limit_time; do
        emu_case "get $key" -- -d $K --get $key
    done
    emu_options setters=0 output=1 load=1 vset=12.00 iset=2.000
    emu_case "get regulation CC" -- -d $K --get regulation
    emu_case "get voltage on" -- -d $K --get voltage
    emu_case "get current on" -- -d $K --get current
    # LABPS3005D: bit 7 means nothing with the output off (api.c's quirk).
    emu_options setters=0 idn=$(emu_hex 'VELLEMANLABPS3005DV2.0') status=80
    emu_case "get LABPS ovp with output off" -- -d $K --get ovp_enabled
    emu_options setters=0 idn=$(emu_hex 'VELLEMANLABPS3005DV2.0') status=c0
    emu_case "get LABPS ovp with output on" -- -d $K --get ovp_enabled
    emu_options setters=0 idn=$(emu_hex 'VELLEMANPS3005DV2.0') status=80
    emu_case "get PS3005D ovp with output off" -- -d $K --get ovp_enabled
    emu_options setters=0 sixth=0
    emu_case "get current_limit no sixth byte" -- -d $K --get current_limit --get voltage
    emu_options setters=0
    emu_case "get current_limit sixth byte" -- -d $K --get current_limit --get voltage

    # A setting has a physical effect: each goes out as asked, alone.
    emu_options
    emu_case "set voltage_target" --no-output -- -d $K -c voltage_target=5.0 --set
    emu_case "set voltage_target max" --no-output -- -d $K -c voltage_target=31 --set
    emu_case "set voltage_target rounds in single precision" --no-output -- -d $K -c voltage_target=1.115 --set
    emu_case "set voltage_target rounds half to even" --no-output -- -d $K -c voltage_target=0.125 --set
    emu_case "set voltage_target -0" --oracle-differs "$ZERO_BUG" --no-output -- -d $K -c voltage_target=-0 --set
    emu_case "set voltage_target too high" --stderr --expect-fail --filter "$A" -- -d $K -c voltage_target=31.01 --set
    emu_case "set voltage_target negative" --stderr --expect-fail --filter "$A" -- -d $K -c voltage_target=-1 --set
    emu_case "set voltage_target nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c voltage_target=nan --set
    emu_case "set current_limit nan" --oracle-differs "$NAN_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c current_limit=nan --set
    emu_case "set current_limit" --no-output -- -d $K -c current_limit=2.225 --set
    emu_case "set current_limit max" --no-output -- -d $K -c current_limit=5.1 --set
    emu_case "set current_limit -0" --oracle-differs "$ZERO_BUG" --no-output -- -d $K -c current_limit=-0 --set
    emu_case "set current_limit too high" --stderr --expect-fail --filter "$A" -- -d $K -c current_limit=5.11 --set
    emu_case "set enabled on" --no-output -- -d $K -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -c enabled=off --set
    emu_case "set ocp on" --no-output -- -d $K -c ocp_enabled=on --set
    emu_case "set ocp off" --no-output -- -d $K -c ocp_enabled=off --set
    emu_case "set ovp on" --no-output -- -d $K -c ovp_enabled=on --set
    emu_case "set ovp off" --no-output -- -d $K -c ovp_enabled=off --set
    # In the order sigrok-cli's hash table has them, so that the tools agree.
    emu_case "set several" --no-output -- -d $K -c current_limit=0.5:voltage_target=3.3:enabled=on --set
    emu_case "set output after the voltage" --oracle-differs "$ORDER_BUG" --no-output -- -d $K -c voltage_target=3.3:enabled=on --set
    emu_case "set then read back" -- -d $K -c voltage_target=3.3 --get voltage_target
    # Nothing is sent unless all of -c is good, and above all no OUT1.
    emu_case "output with a voltage too high" --oracle-bug "$PARTIAL_BUG" --expect-fail --stderr --filter "$A" -- -d $K -c enabled=on:voltage_target=99 --set
    emu_case "output with a voltage that is no number" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr -- -d $K -c enabled=on:voltage_target=abc --set
    emu_case "output with an unknown option" --oracle-bug "$PARTIAL_BUG" --expect-fail --stderr -- -d $K -c enabled=on:volage_target=5 --set
    emu_case "ocp with a malformed boolean" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr -- -d $K -c ocp_enabled=1x --set
    emu_case "output with a malformed boolean" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr -- -d $K -c enabled=1x --set
    emu_case "output with no value" --oracle-differs "$LOOSE_BUG" --expect-fail --stderr -- -d $K -c enabled= --set
    emu_case "output before a limit that is no number" --oracle-bug "$PARTIAL_BUG" --expect-fail --stderr -- -d $K -c enabled=on --samples abc
    emu_options idn=$(emu_hex 'VELLEMANPS3005DV1.3')
    emu_case "set on PS3005D V1.3" --oracle-differs "$ORDER_BUG" --no-output -- -d $K -c enabled=on:voltage_target=3.3 --set
    emu_options idn=$(emu_hex 'KORAD KD6005P V2.2') vmax=61
    emu_case "set KD6005P 60 V" --no-output -- -d $K -c voltage_target=60 --set
    emu_options idn=$(emu_hex 'TENMA 72-2535 V2.1') imax=3.1
    emu_case "set 72-2535 too much current" --stderr --expect-fail --filter "$A" -- -d $K -c current_limit=3.2 --set
    emu_options

    # A capture: IOUT1? VOUT1? STATUS? in turn, a sample counted per VOUT1?.
    emu_options setters=0 output=1 load=10
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 3" -- -d $K --samples 3
    # hwdriver.c's check_key refuses it ahead of the driver, and logs that.
    Z="$A
/^sr: hwdriver: Cannot set/d"
    emu_case "config limit_samples 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_samples=0 --samples 1
    emu_case "config limit_time 0" --stderr --expect-fail --filter "$Z" -- -d $K -c limit_time=0 --samples 1
    emu_case "samples 5 csv" -- -d $K --samples 5 -O csv:header=false:time=false
    emu_case "channel V only" -- -d $K -C V --samples 2
    emu_case "channel I only" -- -d $K -C I --samples 2
    emu_case "channel V only csv" -- -d $K -C V --samples 2 -O csv:header=false:time=false
    emu_options setters=0 idn=$(emu_hex 'VELLEMANPS3005DV1.3') output=1 load=10
    emu_case "samples 3 on PS3005D V1.3" -- -d $K --samples 3
    emu_options setters=0 output=1 load=10 vout=12.34,12.35,00.00,31.00 iout=0.125,5.100,0.000,1.000
    emu_case "readings in turn" -- -d $K --samples 4
    # The status changes under a capture: META lines, as the C has them.
    # The harness strips them unless a case keeps the header.
    emu_options setters=0 status=11,51,11,10,12,31,91
    emu_case "status changes" --keep-header -- -d $K --samples 8 -O analog
    emu_case "status changes default output" --keep-header -- -d $K --samples 8
    # A setting ahead of the capture is in the first status's reckoning.
    emu_options
    # The output goes on just before the start: OUT1, then the queries.
    emu_case "capture with the output on" --keep-header -- -d $K -c enabled=on --samples 1
    emu_case "capture after a set" --keep-header -- -d $K -c enabled=on --samples 3 -O analog
    emu_options status=11,51
    emu_case "capture after a set the device ignores" --keep-header -- -d $K -c enabled=off --samples 2 -O analog
    emu_options setters=0 output=1
    # A time limit: how many polls that is depends on the clock, so both
    # are kept up to the first status of the capture.
    emu_case "time" --filter '3,$d' \
        --tx-filter '/^bulk_in 82 1 -> /{x;s/^/x/;/^xx/{x;q};x}' -- -d $K --time 500ms

    # Answers gone wrong. A garbage value keeps the last good one, a short one
    # reads as far as it goes, and silence is no reading in oxrok, so a capture
    # ends after 5 in a row (C makes up 0, and counts it). A status with no
    # answer reads as 0 in both, as firmware is said to do that.
    emu_options setters=0 output=1 vout=12.34,xx.xx,1.,1.5e1
    emu_case "garbage and short readings" -- -d $K --samples 4
    emu_options setters=0 answers=1
    emu_case "only the ID answered" --oxrok-fails "5 readings with no answer" --oracle-differs "$SILENT_BUG" --no-output -- -d $K --samples 2
    emu_case "only the ID answered get" --oracle-differs "$SILENT_BUG" --expect-fail --stderr -- -d $K --get voltage_target
    emu_case "only the ID answered get enabled" -- -d $K --get enabled
    emu_case "only the ID answered show" --oracle-differs "$SILENT_BUG" --keep-header -- -d $K --show
    emu_options setters=0 answers=7
    emu_case "silent from the capture on" --oxrok-fails "5 readings with no answer" --oracle-differs "$SILENT_BUG" -- -d $K --samples 2
    emu_options setters=0 sixth=0
    emu_case "no sixth byte in the scan" -- -d $K --samples 1

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 output=1 chunk=1
    emu_case "one byte per transfer" -- -d $K --samples 2
    emu_options setters=0 output=1 in-stall=1
    emu_case "a stalled read" --keep-header -- -d $K --scan
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $K --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # An unplug during the scan (a read that finds nothing goes on, the
    # next write fails: api.c's exit_err), and during a capture.
    emu_options setters=0 answers=2
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $K --scan
    emu_options setters=0 answers=8
    emu_case "unplug in a capture" --oxrok-fails "Broken pipe" --oracle-differs "$HUP_BUG" --unplug model --timeout 30 --tx-filter '/^unplug$/q' -- -d $K --samples 3
    # A quiet point: the status goes unanswered, the signal lands in its wait.
    emu_options setters=0 answers=8
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $K --continuous
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=korad-kaxxxxp:conn=$EMU_TTY
}
