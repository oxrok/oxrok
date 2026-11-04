# A Korad KAxxxxP power supply (emu/src/models/korad-kaxxxxp.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=korad-kaxxxxp
EMU_CDC=1
EMU_DRIVER=korad-kaxxxxp
EMU_GCOV=korad-kaxxxxp
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    K=korad-kaxxxxp:conn=$EMU_TTY

    # Every entry of api.c's models[], with the ID strings its table had
    # up to libsigrok d241cc2 (72-2705's is made from its name). A scan
    # asks *IDN?, then IOUT1? ISET1? VOUT1? VSET1? STATUS?, and sets nothing:
    # setters=0 flags a setting, which the .tx would also show.
    for id in "KORADKA3005PV2.0" "KORAD KA3005P V5.5" "KORADKD3005PV2.0" "KORAD KD3005P V6.8" \
        "KORAD KD6005P V2.2" "RND 320-KA3005P V5.5" "RND 320-KD3005P V4.2" "S-LS-31 V2.0" \
        "TENMA 72-2535 V2.1" "TENMA72-2540V2.0" "TENMA 72-2540 V5.2" "TENMA72-2550V2.0" \
        "TENMA 72-2705 V2.0" "TENMA 72-2710 V6.6" "VELLEMANLABPS3005DV2.0" "VELLEMANPS3005DV1.3" \
        "VELLEMANPS3005DV2.0"; do
        korad_options setters=0 idn=$(korad_hex "$id")
        emu_case "scan $id" --keep-header -- -d $K --scan
    done

    # How far the ID check stretches (model_matches, api.c:96-199).
    korad_options setters=0 idn=$(korad_hex 'KORADKA3005PV2.0\001')
    emu_case "scan KA3005P trailing 01" --keep-header -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'KORADKA3005PV2.0\274')
    emu_case "scan KA3005P trailing bc" --keep-header -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'KORAD KA3005P V5.8 SN:12345678')
    emu_case "scan a serial number" --keep-header -- -d $K --scan
    emu_case "show a serial number" --keep-header -- -d $K --show
    korad_options setters=0 idn=$(korad_hex 'KORAD KA3005P SN:1 SN:2')
    emu_case "scan the last SN counts" --keep-header --no-output -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'korad ka3005p v2.0')
    emu_case "scan in lower case" --keep-header --no-output -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'KORAD  KA3005P')
    emu_case "scan with spaces and no version" --keep-header -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'KORADKA3005PV2.0\000junk')
    emu_case "scan cut at a NUL" --keep-header -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'KORADKD3005PV2.0\001')
    emu_case "scan KD3005P has no trailing byte" --keep-header --no-output -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'RND 320-KA3005P V5.5 ')
    emu_case "scan RND with a space after the version" --keep-header -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'RND 320-KA3005P V5.5x')
    emu_case "scan RND with garbage" --keep-header --no-output -- -d $K --scan
    korad_options setters=0 idn=$(korad_hex 'ACME PSU 9000')
    emu_case "scan an unknown ID" --stderr --keep-header -- -d $K --scan
    # A reply longer than the buffer: 48 of it are read, no match.
    korad_options setters=0 idn=$(korad_hex 'KORADKA3005PV2.0 0123456789012345678901234567890123456789')
    emu_case "scan a reply too long" --keep-header --no-output -- -d $K --scan
    # Silence: 100 reads of 60 ms each, then no model.
    korad_options setters=0 idn=none
    emu_case "scan no answer" --keep-header --no-output -- -d $K --scan
    # force_detect stands in for what did not match, or did not answer.
    emu_case "force_detect on silence" --keep-header -- -d $K:force_detect=KORADKD3005PV2.0 --scan
    korad_options setters=0 idn=$(korad_hex 'ACME PSU 9000')
    emu_case "force_detect on an unknown ID" --keep-header -- -d "$K:force_detect=TENMA 72-2535 V2.1" --scan
    emu_case "force_detect unknown too" --keep-header --no-output -- -d $K:force_detect=NOSUCH --scan
    korad_options setters=0
    emu_case "force_detect not needed" --keep-header -- -d $K:force_detect=KORADKD3005PV2.0 --scan
    korad_options setters=0 idn=none
    emu_case "force_detect empty" --keep-header --stderr --expect-fail -- -d "$K:force_detect=" --scan
    korad_options setters=0
    emu_case "no such node" --keep-header --no-output -- -d korad-kaxxxxp:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d korad-kaxxxxp --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d korad-kaxxxxp --show

    # The line: native USB does not mind; behind the RS232 port it must fit.
    emu_case "serialcomm on the native port" --keep-header -- -d $K:serialcomm=19200/8n1 --scan
    korad_options setters=0 uart=9600/8n1
    emu_case "serialcomm as the wiki" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_case "serialcomm misset behind the UART" --keep-header --no-output -- -d $K:serialcomm=19200/8n1 --scan

    # What --show and --get ask for, per range in the table.
    korad_options setters=0
    emu_case "show" --keep-header -- -d $K --show
    korad_options setters=0 idn=$(korad_hex 'KORAD KD6005P V2.2') vmax=61
    emu_case "show KD6005P" --keep-header -- -d $K --show
    korad_options setters=0 idn=$(korad_hex 'TENMA 72-2535 V2.1') imax=3.1
    emu_case "show 72-2535" --keep-header -- -d $K --show
    korad_options setters=0 idn=$(korad_hex 'TENMA72-2550V2.0') vmax=61 imax=3.1
    emu_case "show 72-2550" --keep-header -- -d $K --show
    korad_options setters=0 output=1 ocp=1 ovp=1 load=10
    emu_case "show on, with a load" --keep-header -- -d $K --show
    for key in conn voltage voltage_target current current_limit enabled regulation \
        ovp_enabled ocp_enabled limit_samples limit_time; do
        emu_case "get $key" -- -d $K --get $key
    done
    korad_options setters=0 output=1 load=1 vset=12.00 iset=2.000
    emu_case "get regulation CC" -- -d $K --get regulation
    emu_case "get voltage on" -- -d $K --get voltage
    emu_case "get current on" -- -d $K --get current
    # LABPS3005D: bit 7 means nothing with the output off (api.c's quirk).
    korad_options setters=0 idn=$(korad_hex 'VELLEMANLABPS3005DV2.0') status=80
    emu_case "get LABPS ovp with output off" -- -d $K --get ovp_enabled
    korad_options setters=0 idn=$(korad_hex 'VELLEMANLABPS3005DV2.0') status=c0
    emu_case "get LABPS ovp with output on" -- -d $K --get ovp_enabled
    korad_options setters=0 idn=$(korad_hex 'VELLEMANPS3005DV2.0') status=80
    emu_case "get PS3005D ovp with output off" -- -d $K --get ovp_enabled
    korad_options setters=0 sixth=0
    emu_case "get current_limit no sixth byte" -- -d $K --get current_limit --get voltage
    korad_options setters=0
    emu_case "get current_limit sixth byte" -- -d $K --get current_limit --get voltage

    # A setting has a physical effect: each goes out as asked, alone.
    korad_options
    emu_case "set voltage_target" --no-output -- -d $K -c voltage_target=5.0 --set
    emu_case "set voltage_target max" --no-output -- -d $K -c voltage_target=31 --set
    emu_case "set voltage_target rounds in single precision" --no-output -- -d $K -c voltage_target=1.115 --set
    emu_case "set voltage_target rounds half to even" --no-output -- -d $K -c voltage_target=0.125 --set
    emu_case "set voltage_target -0" --no-output -- -d $K -c voltage_target=-0 --set
    emu_case "set voltage_target too high" --stderr --expect-fail -- -d $K -c voltage_target=31.01 --set
    emu_case "set voltage_target negative" --stderr --expect-fail -- -d $K -c voltage_target=-1 --set
    # api.c's range check lets a NaN through: `VSET1:  nan` goes out.
    emu_case "set voltage_target nan" --no-output -- -d $K -c voltage_target=nan --set
    emu_case "set current_limit nan" --no-output -- -d $K -c current_limit=nan --set
    emu_case "set current_limit" --no-output -- -d $K -c current_limit=2.225 --set
    emu_case "set current_limit max" --no-output -- -d $K -c current_limit=5.1 --set
    emu_case "set current_limit -0" --no-output -- -d $K -c current_limit=-0 --set
    emu_case "set current_limit too high" --stderr --expect-fail -- -d $K -c current_limit=5.11 --set
    emu_case "set enabled on" --no-output -- -d $K -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -c enabled=off --set
    emu_case "set ocp on" --no-output -- -d $K -c ocp_enabled=on --set
    emu_case "set ocp off" --no-output -- -d $K -c ocp_enabled=off --set
    emu_case "set ovp on" --no-output -- -d $K -c ovp_enabled=on --set
    emu_case "set ovp off" --no-output -- -d $K -c ovp_enabled=off --set
    emu_case "set several in order" --no-output -- -d $K -c voltage_target=3.3:current_limit=0.5:enabled=on --set
    emu_case "set then read back" -- -d $K -c voltage_target=3.3 --get voltage_target
    korad_options idn=$(korad_hex 'VELLEMANPS3005DV1.3')
    emu_case "set on PS3005D V1.3" --no-output -- -d $K -c voltage_target=3.3:enabled=on --set
    korad_options idn=$(korad_hex 'KORAD KD6005P V2.2') vmax=61
    emu_case "set KD6005P 60 V" --no-output -- -d $K -c voltage_target=60 --set
    korad_options idn=$(korad_hex 'TENMA 72-2535 V2.1') imax=3.1
    emu_case "set 72-2535 too much current" --stderr --expect-fail -- -d $K -c current_limit=3.2 --set
    korad_options

    # A capture: IOUT1? VOUT1? STATUS? in turn, a sample counted per VOUT1?.
    korad_options setters=0 output=1 load=10
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 3" -- -d $K --samples 3
    emu_case "samples 5 csv" -- -d $K --samples 5 -O csv:header=false:time=false
    emu_case "channel V only" -- -d $K -C V --samples 2
    emu_case "channel I only" -- -d $K -C I --samples 2
    emu_case "channel V only csv" -- -d $K -C V --samples 2 -O csv:header=false:time=false
    korad_options setters=0 idn=$(korad_hex 'VELLEMANPS3005DV1.3') output=1 load=10
    emu_case "samples 3 on PS3005D V1.3" -- -d $K --samples 3
    korad_options setters=0 output=1 load=10 vout=12.34,12.35,00.00,31.00 iout=0.125,5.100,0.000,1.000
    emu_case "readings in turn" -- -d $K --samples 4
    # The status changes under a capture: META lines, as the C has them.
    korad_options setters=0 status=11,51,11,10,12,31,91
    emu_case "status changes" --keep-header -- -d $K --samples 8
    # A setting ahead of the capture is in the first status's reckoning.
    korad_options
    emu_case "capture after a set" --keep-header -- -d $K -c enabled=on --samples 3
    korad_options status=11,51
    emu_case "capture after a set the device ignores" --keep-header -- -d $K -c enabled=off --samples 2
    korad_options setters=0 output=1
    # A time limit: how many polls that is depends on the clock, so both
    # are kept up to the first status of the capture.
    emu_case "time" --filter '3,$d' \
        --tx-filter '/^bulk_in 82 1 -> /{x;s/^/x/;/^xx/{x;q};x}' -- -d $K --time 500ms

    # Answers gone wrong. A garbage value keeps the last good one, silence
    # reads as 0 (sr_atof_ascii takes an empty string as 0.0), a short one
    # as far as it goes.
    korad_options setters=0 output=1 vout=12.34,xx.xx,1.,1.5e1
    emu_case "garbage and short readings" -- -d $K --samples 4
    korad_options setters=0 answers=1
    emu_case "only the ID answered" -- -d $K --samples 2
    emu_case "only the ID answered get" -- -d $K --get voltage_target --get enabled
    korad_options setters=0 answers=7
    emu_case "silent from the capture on" -- -d $K --samples 2
    korad_options setters=0 sixth=0
    emu_case "no sixth byte in the scan" -- -d $K --samples 1

    # The USB side misbehaving, the shell's options.
    korad_options setters=0 output=1 chunk=1
    emu_case "one byte per transfer" -- -d $K --samples 2
    korad_options setters=0 output=1 in-stall=1
    emu_case "a stalled read" -- -d $K --samples 2
    korad_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $K --scan
    korad_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # An unplug during the scan (a read that finds nothing goes on, the
    # next write fails: api.c's exit_err), and during a capture.
    korad_options setters=0 answers=2
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $K --scan
    korad_options setters=0 answers=8
    emu_case "unplug in a capture" --unplug model --timeout 30 --tx-filter '/^unplug$/q' -- -d $K --samples 3
    korad_options
}

# The model's options for the cases that follow (see
# tools/emu/ftdi-la.sh); none restores EMU_OPTIONS.
korad_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "korad-kaxxxxp: the emulator refused '$*'"
}

# TEXT as hex, printf's backslash escapes included.
korad_hex() {
    printf "$1" | od -An -tx1 | tr -d ' \n'
}
