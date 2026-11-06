# An EDF Teleinfo meter (emu/src/models/teleinfo.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=teleinfo
EMU_CDC=1
EMU_DRIVER=teleinfo
EMU_GCOV=teleinfo
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# api.c:79 asks serial_stream_detect for a packet of 292 bytes with a buffer of
# 292, and serial.c:833 has wanted a buffer of two packets since d03815a0 (2019,
# `maxlen < packet_size / 2` became `* 2`, in 0.5.2): every scan fails, so stock
# sigrok-cli finds no device. So a "full" case below means against a patched C,
# and all cases but the "stock" ones at the end run the C driver with that one
# check taken out in memory, by tools/emu/teleinfo-scan.c, which is set on the exec
# of this model's tools only, through wrappers that SIGROK_CLI and
# EMU_LIBSIGROK_RUN name for the length of emu_cases. Neither oxrok nor the harness
# runs under it. This is how the scan ran up to 0.5.1: read 292 bytes, then look.
teleinfo_scan_fixed() {
    TI_STOCK_CLI=$SIGROK_CLI
    TI_STOCK_RUN=$EMU_LIBSIGROK_RUN
    [ "$no_oracle" = 1 ] && return
    cc -shared -fPIC -o "$TMP/teleinfo-scan.so" "$TOOLS/emu/teleinfo-scan.c" -ldl 2>"$TMP/teleinfo-scan.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    # The runner's own check compares what sigrok-cli and it load, so it is
    # built ahead of the wrapper.
    emu_libsigrok_run || { emu_fail "$EMU_MODEL: $EMU_LIBSIGROK_ERR"; return; }
    for w in cli:"$(command -v "$SIGROK_CLI")" run:"$EMU_LIBSIGROK_RUN"; do
        printf '%s\n' '#!/bin/sh' \
            "LD_PRELOAD=$TMP/teleinfo-scan.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec ${w#*:} \"\$@\"" \
            >"$TMP/teleinfo-${w%%:*}"
        chmod +x "$TMP/teleinfo-${w%%:*}"
    done
    SIGROK_CLI=$TMP/teleinfo-cli
    EMU_LIBSIGROK_RUN=$TMP/teleinfo-run
}

emu_conns() {
    K=teleinfo:conn=$EMU_TTY
}

# A BASE frame of 152 bytes, so two of them hold a scan's 292.
TI_BASE="ADCO:031428097115+OPTARIF:BASE+ISOUSC:30+BASE:000012000+PTEC:TH..+IINST:002+IMAX:030+PAPP:00460+HHPHC:A+MOTDETAT:000000"
# The group that ends a sample limit: it counts, and nothing follows it to be
# lost by how reads fall.
TI_END="ADCO:031428097115"

# ti_sweep NAME SAMPLES FRAME...: a capture of these frames, in turn, behind the
# scan's two; SAMPLES is how many ADCO groups they hold, the last at their end.
# TI_FLAGS goes to the case: --no-output where the C gives nothing.
ti_sweep() {
    _name=$1
    _samples=$2
    shift 2
    _list=$TI_BASE,$TI_BASE
    for _f in "$@"; do
        _list=$_list,$_f
    done
    emu_options period=5 scan-burst=2 burst=$# frame=$_list
    emu_case "$_name" $TI_FLAGS -- -d $K --samples $_samples
}

# ti_raw TEXT: a frame= group of these bytes, a printf format.
ti_raw() {
    printf '=%s' "$(emu_hex "$1")"
}

# ti_lines NAME TEXT: a capture of one read of these bytes, a printf format.
ti_lines() {
    ti_sweep "$1" 1 "$(ti_raw "$2")" "$TI_END"
}

# ti_scan NAME FRAME...: a scan that hears these frames, in turn.
ti_scan() {
    _name=$1
    shift
    _list=
    for _f in "$@"; do
        _list=$_list${_list:+,}$_f
    done
    emu_options period=5 burst=$# frame=$_list
    emu_case "$_name" --keep-header $TI_FLAGS -- -d $K --scan
}

emu_cases() {
    teleinfo_scan_fixed
    emu_conns
    TI_FLAGS=
    # What a time limit gets is the clock's: frames repeat here, and the filter
    # keeps the first of them, with the transfers' lengths folded.
    N='s/^bulk_in 82 [0-9]+ -> .*$/bulk_in 82 STREAM/'
    # 300 bytes in no group, to fill a scan.
    Z=$(ti_raw "%0300d")

    # The keys. Default frames, two to a scan.
    emu_options period=5 scan-burst=2 burst=0
    emu_case "show" --keep-header -- -d $K --show
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "get limit_samples" --expect-fail --stderr -- -d $K --get limit_samples
    emu_case "get limit_time" --expect-fail --stderr -- -d $K --get limit_time
    emu_case "get conn" --expect-fail --stderr -- -d $K --get conn
    emu_case "set limit_samples" --no-output -- -d $K --set -c limit_samples=1
    emu_case "set something else" --expect-fail --stderr -- -d $K --set -c samplerate=1
    emu_case "no conn" --keep-header --no-output -- -d teleinfo --scan
    emu_case "no such node" --keep-header --no-output -- -d teleinfo:conn=/dev/ttyACM97 --scan
    emu_options

    # The channels follow the option a frame says.
    emu_options period=5 scan-burst=2 optarif=HC..
    emu_case "scan HC.." --keep-header -- -d $K --scan
    emu_options period=5 scan-burst=2 optarif=EJP.
    emu_case "scan EJP." --keep-header -- -d $K --scan
    emu_options period=5 scan-burst=2 'optarif=BBR('
    emu_case "scan BBR(" --keep-header -- -d $K --scan

    # The line.
    emu_options period=5 scan-burst=2 burst=0
    emu_case "wrong baud" --keep-header --no-output -- -d $K:serialcomm=9600/7e1 --scan
    emu_case "wrong frame" --keep-header --no-output -- -d $K:serialcomm=1200/8n1 --scan
    emu_case "no such serialcomm" --keep-header --no-output --tx-filter '/^ctl_out/d' -- -d $K:serialcomm=fast --scan
    emu_options period=5 scan-burst=2 uart=9600/7e1
    emu_case "scan at another rate" --keep-header -- -d $K:serialcomm=9600/7e1 --scan

    # What a scan's 292 bytes hold. Nothing makes a device but a known option in
    # a group whose control is right.
    ti_scan "scan option alone" "OPTARIF:BASE+$Z"
    ti_scan "scan the last option wins" "OPTARIF:BASE+OPTARIF:HC..+$Z"
    ti_scan "scan a last option that is cut" "OPTARIF:BASE+$(ti_raw "%0264d")+OPTARIF:HC.."
    ti_scan "scan BBR alone" "OPTARIF:BBR+$Z"
    # An option over 4 bytes overruns C's optarif[5] (protocol.c:95), which goes
    # unnoticed on this build.
    ti_scan "scan BBR too long" "OPTARIF:BBRABCD+$Z"
    TI_FLAGS=--no-output
    ti_scan "scan no option" "ADCO:031428097115+$Z"
    ti_scan "scan unknown option" "OPTARIF:TEMP+$Z"
    ti_scan "scan wrong control" "OPTARIF:BASE:21+$Z"
    ti_scan "scan a last option unknown" "OPTARIF:BASE+OPTARIF:TEMP+$Z"
    ti_scan "scan a short option" "OPTARIF:HC+$Z"
    ti_scan "scan option too long" "OPTARIF:BASEX+$Z"
    ti_scan "scan other case" "OPTARIF:base+$Z"
    TI_FLAGS=
    # 3 s of a scan that hears too little, and of one that hears nothing.
    emu_options period=5 burst=1 frame=OPTARIF:BASE
    emu_case "scan option and then silence" --keep-header --no-output -- -d $K --scan
    emu_options burst=0
    emu_case "scan nothing" --keep-header --no-output -- -d $K --scan
    emu_options

    # A capture, plain. What follows a limit in another read is lost, so ADCO
    # comes last here and the frame fits one transfer.
    ti_sweep "samples 1" 1 "BASE:000012345+PTEC:TH..+IINST:002+PAPP:00460+$TI_END"
    ti_sweep "samples 2" 2 "BASE:000012345+$TI_END" "BASE:000012346+$TI_END"
    ti_sweep "a long frame, then the end" 2 "$TI_BASE" "$TI_END"
    # Two in one read: the limit waits for the read's end.
    ti_sweep "a read of two" 1 "$TI_END+BASE:000000001+$TI_END+BASE:000000002"
    ti_sweep "IINST and PAPP alone" 1 "IINST:002+PAPP:00460+$TI_END"
    emu_case "channels IINST" -- -d $K -C IINST --samples 1
    emu_case "channels PAPP and P" -- -d $K -C PAPP,P --samples 1
    emu_case "channels P" --no-output -- -d $K -C P --samples 1
    emu_case "renamed" --no-output -- -d $K -C IINST=amps --samples 1
    emu_case "no such channel" --expect-fail --stderr -- -d $K -C NOPE --samples 1

    # With P, C's csv prints a heap value for the column no reading fills
    # (csv.c:187, :437), so this is without it.
    emu_options period=5 scan-burst=2 burst=1 frame=$TI_BASE,$TI_BASE,BASE:000012345+IINST:002+PAPP:00460+$TI_END
    emu_case "csv without P" -- -d $K -C BASE,IINST,PAPP --samples 1 -O csv:header=false:time=false

    # The labels. Each with its value, unit and channel.
    _hc="OPTARIF:HC..+ISOUSC:30+HCHC:000001000+HCHP:000002000+PTEC:HP..+IINST:002+PAPP:00460+HHPHC:A+MOTDETAT:000000+ADCO:031428097115"
    emu_options period=5 scan-burst=4 burst=1 frame=$_hc,$_hc,$_hc,$_hc,HCHC:000000011+HCHP:000000012+BASE:000000013+EJPHN:000000014+EJPHPM:000000015+IINST:016+PAPP:00017+$TI_END
    emu_case "labels HC" -- -d $K --samples 1
    _ej="OPTARIF:EJP.+ISOUSC:30+EJPHN:000003000+EJPHPM:000004000+PTEC:HN..+IINST:002+PAPP:00460+HHPHC:A+MOTDETAT:000000+ADCO:031428097115"
    emu_options period=5 scan-burst=4 burst=1 frame=$_ej,$_ej,$_ej,$_ej,HCHC:000000011+HCHP:000000012+BASE:000000013+EJPHN:000000014+EJPHPM:000000015+IINST:016+PAPP:00017+$TI_END
    emu_case "labels EJP" -- -d $K --samples 1
    _bb="OPTARIF:BBR(+ISOUSC:30+BBRHCJB:000005000+BBRHPJB:000006000+BBRHCJW:000007000+BBRHPJW:000008000+BBRHCJR:000009000+BBRHPJR:000010000+PTEC:HPJB+DEMAIN:----+IINST:002+PAPP:00460+MOTDETAT:000000+ADCO:031428097115"
    emu_options period=5 scan-burst=2 burst=1 frame=$_bb,$_bb,BBRHCJB:000000020+BBRHPJB:000000022+BBRHCJW:000000023+BBRHPJW:000000024+BBRHCJR:000000025+BBRHPJR:000000026+$TI_END
    emu_case "labels BBR" -- -d $K --samples 1
    emu_options
    TI_FLAGS=--no-output
    ti_sweep "labels of other options" 1 "HCHC:000000011+HCHP:000000012+EJPHN:000000014+EJPHPM:000000015+BBRHCJB:000000021+$TI_END"
    ti_sweep "labels nobody reads" 1 "OPTARIF:BASE+ISOUSC:30+PTEC:TH..+DEMAIN:----+IMAX:030+ADPS:031+HHPHC:A+MOTDETAT:000000+P:000000001+$TI_END"
    ti_sweep "other case" 1 "base:000000001+Base:000000002+$TI_END"
    TI_FLAGS=

    # The values: what atoi makes, as a float.
    ti_sweep "values" 1 "BASE:000000000+BASE:000000001+BASE:016777216+BASE:016777217+$TI_END"
    ti_sweep "values large" 1 "BASE:123456789+BASE:999999999+BASE:2147483647+BASE:2147483648+$TI_END"
    ti_sweep "values wider" 1 "BASE:4294967296+BASE:9999999999+BASE:999999999999+$TI_END"
    ti_lines "values signed" '\nBASE +12 )\r\nBASE -12 +\r\nIINST -1 %%\r\nPAPP --1 <\r'
    ti_sweep "values not numbers" 1 "BASE:ABC+BASE:12A+BASE:1.5+BASE:0x10+BASE:....+$TI_END"
    ti_lines "a space inside a value" '\nBASE 12 34 >\r\nBASE 000000003 N\r'

    # What makes a group. Of no consequence: STX and ETX, and what comes before a
    # group's LF.
    ti_lines "a lowercase control" '\nBASE 000000001 l\r\nBASE 000000001 L\r'
    # These groups' sums' low six bits are 0: a control of a space, which the
    # specification allows, and the " %%c" of protocol.c:134 steps over it.
    ti_sweep "control of a space" 1 "BASE:000000399+IINST:009+BASE:000000002+$TI_END"
    ti_lines "white space" '\n\nBASE 000000001 L\r\n BASE 000000002 M\r\nBASE  000000003 N\r\nBASE 000000001  L\r\nBASE\t000000002\tM\r\n\tBASE\t000000003 N\r'
    ti_lines "other separators" '\nBASE\v000000001\fL\r\nBASE;000000001;L\r'
    ti_lines "junk around groups" '\002\003\004xx\nBASE 000000001 L\rxx\nBASE 000000002 M\r\003\rxx'
    ti_lines "a CR ahead of the group" '\r\r\nBASE 000000001 L\r\r\r'
    ti_lines "a group across a CR" '\nBASE\r\n000000001 L\r\nBASE 000000002 M\r'
    ti_lines "a tail behind the control" '\nBASE 000000001 Lx\r\nBASE 000000001 L \r\nBASE 000000002 M\r'
    # Widths: %%8s and %%13s cut a field, and the rest is the next one.
    ti_lines "label of 8" '\nBBRHCJBX 000000001 6\r\nBASE 000000002 M\r'
    ti_lines "label of 9" '\nBBRHCJBXY 000000001 6\r\nBASE 000000002 M\r'
    ti_lines "label cut into data" '\nBASEBASEB 8\r\nBASE 000000002 M\r'
    ti_sweep "data of 12" 1 "BASE:000000000012+$TI_END"
    ti_sweep "data of 14" 1 "BASE:00000000000014+BASE:000000002+$TI_END"
    ti_sweep "empty data" 1 "BASE:+BASE:000000002+$TI_END"
    # A NUL ends a string, and bytes over 127 sum as signed chars.
    ti_lines "a NUL" '\nBASE 000\00000001 L\r\nBASE 000000002 M\r'
    ti_lines "a high byte in data" '\nBASE 000000001\351 5\r\nBASE 000000001\351 B\r\nBASE 000000002 M\r'
    ti_lines "a high control" '\nBASE 000000001 \314\r\nBASE 000000002 M\r'
    TI_FLAGS=--no-output
    ti_sweep "wrong controls" 1 "BASE:000000001:21+BASE:000000002:7f+BASE:000000003:00+$TI_END"
    ti_lines "no fields" '\n\r\n \r\n  \r\nBASE\r\nBASE 1\r\nBASE 000000001\r\nBASE 000000001 \r'
    # The second standard, 9600 bd's: tabs, and a control over the tab too.
    ti_lines "a control of the second kind" '\nBASE\t000000001\t5\r'
    # data[13] has no room for %%13s's NUL (protocol.c:132), which clears the label
    # on this build: a group of 13 never counts.
    ti_sweep "data of 13" 1 "BASE:0000000000013+IINST:0000000000013+$TI_END"
    ti_lines "a high byte in a label" '\nBASE\351 000000001 5\r\nBASE\351 000000001 B\r'
    TI_FLAGS=

    # An interruption, as the specification has it.
    emu_options period=5 scan-burst=2 burst=3 frame=$TI_BASE,$TI_BASE,$TI_BASE,$TI_BASE,$TI_END eot=3
    emu_case "interrupted frame" -- -d $K --samples 2
    emu_options

    # A full buffer ends the capture with no end, 256 bytes with no group in them.
    ti_sweep "a group ends at byte 256" 1 "$(ti_raw "%0237d")+BASE:000000001" "$TI_END"
    TI_FLAGS=--no-output
    ti_sweep "no group in 256" 1 "$Z"
    ti_sweep "a group that ends at byte 257" 1 "$(ti_raw "%0238d")+BASE:000000001" "$TI_END"
    ti_sweep "a LF and no CR" 1 "$(ti_raw "\n%0300d")"
    ti_sweep "a head with no end" 1 "$(ti_raw "\nBASE %0300d")"
    TI_FLAGS=

    # How reads fall.
    emu_options period=5 scan-burst=2 burst=2 chunk=7 frame=$TI_BASE,$TI_BASE,$TI_BASE,$TI_END
    emu_case "cut in reads of 7" -- -d $K --samples 2
    emu_options

    # A time limit: how many frames came is the clock's. The one free stream, with
    # lengths folded. It ends in C only because bytes come after the limit.
    emu_options frame=$TI_BASE
    emu_case "time" --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --time 300ms
    emu_case "time and samples" --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --time 300ms --samples 100
    emu_options

    # Silence, where C has no end: protocol.c:193 returns before any limit.
    emu_options period=5 scan-burst=2 burst=1 frame=$TI_BASE,$TI_BASE,$TI_BASE
    emu_case "lib silence and samples" --libsigrok --hangs --timeout 3 -- -d $K --samples 100
    emu_case "lib silence and time" --libsigrok --hangs --timeout 3 -- -d $K --time 200ms
    emu_case "unplug in a capture" --hangs --timeout 5 --unplug model -- -d $K --samples 100
    emu_options

    # The cable.
    emu_options period=5 scan-burst=2 stall=lines
    emu_case "stall lines" --keep-header --no-output --stderr -- -d $K --scan
    emu_options period=5 scan-burst=2 stall=line
    emu_case "stall line" --keep-header --no-output -- -d $K --scan
    emu_options

    # Stock libsigrok: no scan finds it.
    SIGROK_CLI=$TI_STOCK_CLI
    EMU_LIBSIGROK_RUN=$TI_STOCK_RUN
    emu_options period=5 scan-burst=2
    emu_case "stock scan" --keep-header --no-output -- -d $K --scan
    emu_case "stock capture" --expect-fail --stderr -- -d $K --samples 1
    emu_options
}
