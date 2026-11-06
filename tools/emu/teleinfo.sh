# An EDF Teleinfo meter (emu/src/models/teleinfo.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=teleinfo
EMU_CDC=1
EMU_DRIVER=teleinfo
EMU_FULL=$EMU_DRIVER
EMU_GCOV=teleinfo
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# api.c:79 asks serial_stream_detect for a packet of 292 bytes with a buffer of
# 292, and serial.c:833 has wanted a buffer of two packets since d03815a0 (2019,
# `maxlen < packet_size / 2` became `* 2`, in 0.5.2): stock sigrok-cli finds no
# device. So the oracle carries tools/oracle-patches/libsigrok-teleinfo-scan.patch,
# and every case here compares against that C. It looks at byte 292, as 0.5.1 did.

emu_conns() {
    K=teleinfo:conn=$EMU_TTY
}

# A BASE frame of 152 bytes, so two of them hold a scan's 292.
TI_BASE="ADCO:031428097115+OPTARIF:BASE+ISOUSC:30+BASE:000012000+PTEC:TH..+IINST:002+IMAX:030+PAPP:00460+HHPHC:A+MOTDETAT:000000"
# The group that ends a sample limit: it counts, and nothing follows it to be
# lost by how reads fall.
TI_END="ADCO:031428097115"

# C counts a sample at ADCO and looks at its limit after each read; oxrok at the
# frame's end, inside a read. So where both must agree, ADCO comes last here, with
# ETX in the same transfer, and "a frame in two reads" shows what differs.

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
    emu_case "$_name" $TI_FLAGS ${TI_WHY:+"$TI_WHY"} -- -d $K --samples $_samples
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
    emu_conns
    TI_FLAGS=
    # What a time limit gets is the clock's: frames repeat here, and the filter
    # keeps the first of them, with the transfers' lengths folded.
    N='s/^bulk_in 82 [0-9]+ -> .*$/bulk_in 82 STREAM/'
    NOT_A_READING="protocol.c:91's atoi makes a value of any data, and a flipped bit 6 passes the control, so C reports 0 Wh or the digits ahead of the first other byte; oxrok drops a group whose data is not all digits at its width, and warns"
    CROSS="protocol.c:134's sscanf reads across the group's CR into what follows; oxrok ends a group at its first CR"
    WHOLE="protocol.c:99-100 counts at ADCO, a frame's first group, and :222 looks after each read, so C ends with what that read held; oxrok counts a sample at ETX or the next ADCO, and gives whole frames"
    STANDARD="sigrok-cli says nothing of a scan that heard bytes and no known option; oxrok warns, and names the standard form when it sees one"
    SPACE="protocol.c:134's sscanf has ' %c', whose space steps over a control that is a space, so the CR is read as the control and the group is dropped, where Enedis-NOI-CPT_02E 1.5.2's arithmetic gives 0x20 and LibTeleinfo's parser takes it; oxrok takes the group"
    RENAME="protocol.c:53,68 finds a channel by name for each reading, so a channel with another name gets none; oxrok keeps its readings"
    HUP="a port that hung up gives revents beyond G_IO_IN, and protocol.c:193 takes that as nothing to do, for ever; oxrok's failed read ends the capture"
    TIME_HANG="protocol.c:193 returns at a read's timeout ahead of the limit's check at :222, so a time limit never ends a capture on a silent meter; oxrok ends it"
    LATE_LIMIT="protocol.c:193 and :222 look at the clock only after bytes came, so a time limit that ends before the first bytes sends those; oxrok ends at the limit"
    NO_BOUND="protocol.c:193 returns at a read's timeout, so a meter that went quiet holds a sample limit for ever; oxrok ends such a capture after 10 s of no bytes"
    USAGE="neither tool starts a capture with no limit: sigrok-cli prints its usage, oxrok its own; the first line is enough to show which"
    SWAP="sigrok-cli's parsers.c:139-149 renames as it reads, so IINST=PAPP makes two channels of that name, and PAPP=IINST finds the first, the same one, and renames it back: one channel, enabled alone; oxrok reads the names as they stood, so the two swap"
    CSV_HEAP="csv.c:187's g_malloc leaves the labels unset and nothing sets P's, as no reading fills it, so C prints what the heap holds, and with this perturbation dies of it; oxrok's is empty"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
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
    emu_case "no limit" --oracle-differs "$USAGE" --filter '2,$d' --timeout 5 -- -d $K -O analog
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
    # What each says of it. The standard form, whose groups have tabs and a control
    # over the tab too, at its 9600 bd.
    emu_options period=5 burst=1 frame=ADCO:031428097115+$Z
    emu_case "scan no option warns" --oracle-differs "$STANDARD" --stderr --keep-header -- -d $K --scan
    emu_options period=5 burst=1 uart=9600/7e1 frame=$(ti_raw '\nADSC\t041876097115\t>\r\nVTIC\t02\tJ\r\nLTARF\t      BASE      \tF\r\nEAST\t000012000\tR\r')+$Z
    emu_case "scan of the standard form" --oracle-differs "$STANDARD" --stderr --keep-header -- -d $K:serialcomm=9600/7e1 --scan
    # 3 s of a scan that hears too little, and of one that hears nothing.
    emu_options period=5 burst=1 frame=OPTARIF:BASE
    emu_case "scan option and then silence" --keep-header --no-output -- -d $K --scan
    emu_options burst=0
    emu_case "scan nothing" --keep-header --no-output -- -d $K --scan
    emu_options

    # A capture, plain.
    ti_sweep "samples 1" 1 "BASE:000012345+PTEC:TH..+IINST:002+PAPP:00460+$TI_END"
    ti_sweep "samples 2" 2 "BASE:000012345+$TI_END" "BASE:000012346+$TI_END"
    ti_sweep "a long frame, then the end" 2 "$TI_BASE" "$TI_END"
    # Two in one read: C's limit waits for the read's end, and oxrok's for the frame's.
    TI_FLAGS=--oracle-differs
    TI_WHY=$WHOLE
    ti_sweep "a read of two" 1 "$TI_END+BASE:000000001+$TI_END+BASE:000000002"
    TI_WHY=
    TI_FLAGS=
    # A frame whose reads come 500 ms apart, ADCO alone in the first. Bytes at the
    # line's real rate come so, and here C stops with nothing.
    emu_options period=25 scan-burst=2 burst=2 packet=$(emu_hex '\002\nADCO 031428097115 @\r\nOPTARIF BASE 0\r\nISOUSC 30 9\r\nBASE 000012000 N\r\nPTEC TH.. $\r\nIINST 002 Y\r\nIMAX 030 B\r\nPAPP 00460 +\r\nHHPHC A ,\r\nMOTDETAT 000000 B\r\003'),$(emu_hex '\002\nADCO 031428097115 @\r\nOPTARIF BASE 0\r\nISOUSC 30 9\r\nBASE 000012000 N\r\nPTEC TH.. $\r\nIINST 002 Y\r\nIMAX 030 B\r\nPAPP 00460 +\r\nHHPHC A ,\r\nMOTDETAT 000000 B\r\003'),$(emu_hex '\002\nADCO 031428097115 @\r'),$(emu_hex '\nBASE 000000001 L\r\nIINST 002 Y\r\003')
    emu_case "a frame in two reads" --oracle-differs "$WHOLE" -- -d $K --samples 1
    ti_sweep "IINST and PAPP alone" 1 "IINST:002+PAPP:00460+$TI_END"
    emu_case "channels IINST" -- -d $K -C IINST --samples 1
    emu_case "swapped names" --oracle-differs "$SWAP" -- -d $K -C IINST=PAPP,PAPP=IINST --samples 1
    # sigrok-cli and oxrok word this refusal differently on every driver: the filter
    # gives C's line oxrok's words, and leaves oxrok's alone.
    emu_case "no such channel" --expect-fail --stderr --filter 's/^unknown channel /Failed to set channels: invalid argument: &/' -- -d $K -C NOPE --samples 1
    emu_case "csv with P" --oracle-bug "$CSV_HEAP" --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 -- -d $K --samples 1 -O csv:header=false:time=false
    emu_case "samples 0 with a time limit" --expect-fail --stderr --timeout 20 -- -d $K --samples 0 --time 500ms
    emu_case "channels PAPP and P" -- -d $K -C PAPP,P --samples 1
    emu_case "channels P" --no-output -- -d $K -C P --samples 1
    emu_case "renamed" --oracle-differs "$RENAME" -- -d $K -C IINST=amps,PAPP --samples 1

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
    ti_sweep "values large" 1 "BASE:123456789+BASE:999999999+$TI_END"
    ti_sweep "an index of 8" 1 "BASE:00012345+HCHC:00012345+$TI_END"
    TI_FLAGS="--no-output --oracle-differs"
    TI_WHY=$NOT_A_READING
    ti_sweep "values wider" 1 "BASE:2147483647+BASE:4294967296+BASE:9999999999+BASE:999999999999+$TI_END"
    ti_lines "values signed" '\nBASE +12 )\r\nBASE -12 +\r\nIINST -1 %%\r\nPAPP --1 <\r'
    ti_sweep "values not numbers" 1 "BASE:ABC+BASE:12A+BASE:1.5+BASE:0x10+BASE:....+$TI_END"
    ti_sweep "values of another width" 1 "BASE:1234567+IINST:12+IINST:0012+PAPP:0460+PAPP:000460+$TI_END"
    # One flipped bit 6: the control still passes, and C reports 0 Wh, 1 Wh and 0 A.
    ti_lines "a flipped bit in a value" '\nBASE p00012000 N\r\nBASE 000012q00 O\r\nIINST 0q2 Z\r'
    TI_FLAGS="--stderr --oracle-differs"
    ti_sweep "values not numbers warn" 1 "BASE:12A+$TI_END"
    TI_WHY=
    TI_FLAGS=
    ti_lines "a space inside a value" '\nBASE 12 34 >\r\nBASE 000000003 N\r'

    # What makes a group. Of no consequence: STX and ETX, and what comes before a
    # group's LF.
    ti_lines "a lowercase control" '\nBASE 000000001 l\r\nBASE 000000001 L\r'
    # These groups' sums' low six bits are 0: a control of a space, which the
    # specification allows, and the " %%c" of protocol.c:134 steps over it.
    TI_FLAGS=--oracle-differs
    TI_WHY=$SPACE
    ti_sweep "control of a space" 1 "BASE:000000399+IINST:009+BASE:000000002+$TI_END"
    TI_WHY=
    TI_FLAGS=
    ti_lines "white space" '\n\nBASE 000000001 L\r\n BASE 000000002 M\r\nBASE  000000003 N\r\nBASE 000000001  L\r\nBASE\t000000002\tM\r\n\tBASE\t000000003 N\r'
    ti_lines "other separators" '\nBASE\v000000001\fL\r\nBASE;000000001;L\r'
    ti_lines "junk around groups" '\002\003\004xx\nBASE 000000001 L\rxx\nBASE 000000002 M\r\003\rxx'
    ti_lines "a CR ahead of the group" '\r\r\nBASE 000000001 L\r\r\r'
    TI_FLAGS=--oracle-differs
    TI_WHY=$CROSS
    ti_lines "a group across a CR" '\nBASE\r\n000000001 L\r\nBASE 000000002 M\r'
    TI_WHY=
    TI_FLAGS=
    ti_lines "a LF inside a group" '\nBASE 0000\n00001 L\r\nBASE 000000002 M\r'
    ti_lines "a tail behind the control" '\nBASE 000000001 Lx\r\nBASE 000000001 L \r\nBASE 000000002 M\r'
    # Widths: %%8s and %%13s cut a field, and the rest is the next one.
    ti_lines "label of 8" '\nBBRHCJBX 000000001 6\r\nBASE 000000002 M\r'
    ti_lines "label of 9" '\nBBRHCJBXY 000000001 6\r\nBASE 000000002 M\r'
    ti_lines "label cut into data" '\nBASEBASEB 8\r\nBASE 000000002 M\r'
    TI_FLAGS=--oracle-differs
    TI_WHY=$NOT_A_READING
    ti_sweep "data of 12" 1 "BASE:000000000012+BASE:000000001+$TI_END"
    ti_lines "a high byte in data" '\nBASE 000000001\351 5\r\nBASE 000000001\351 B\r\nBASE 000000002 M\r'
    TI_WHY=
    TI_FLAGS=
    ti_sweep "data of 14" 1 "BASE:00000000000014+BASE:000000002+$TI_END"
    ti_sweep "empty data" 1 "BASE:+BASE:000000002+$TI_END"
    # A NUL ends a string, and bytes over 127 sum as signed chars.
    ti_lines "a NUL" '\nBASE 000\00000001 L\r\nBASE 000000002 M\r'
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
    TI_FLAGS="--no-output --oxrok-fails"
    TI_WHY="256 bytes came with no group in them"
    ti_sweep "no group in 256" 1 "$Z"
    ti_sweep "a group that ends at byte 257" 1 "$(ti_raw "%0238d")+BASE:000000001" "$TI_END"
    ti_sweep "a LF and no CR" 1 "$(ti_raw "\n%0300d")"
    ti_sweep "a head with no end" 1 "$(ti_raw "\nBASE %0300d")"
    TI_WHY=
    TI_FLAGS=

    # How reads fall.
    emu_options period=5 scan-burst=2 burst=2 chunk=7 frame=$TI_BASE,$TI_BASE,$TI_BASE,$TI_END
    emu_case "cut in reads of 7" -- -d $K --samples 2
    emu_options

    # A time limit: how many frames came is the clock's. These are the free
    # streams, a frame a second, with lengths folded, and the first line of output,
    # as the rest of a frame may fall behind a limit. C ends only because bytes
    # come after the limit, so 3 s is past the first frame.
    emu_options frame=$TI_BASE
    emu_case "free stream, samples" --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --samples 1
    emu_case "free stream, time" --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --time 3s
    # A limit that runs out ahead of the first frame, held 150 offers, about 3 s, so
    # no stall of oxrok's shows it a frame, and C waits for that frame.
    emu_options period=5 scan-burst=2 late=150 frame=$TI_BASE
    emu_case "time" --oracle-differs "$LATE_LIMIT" --no-output --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --time 300ms
    emu_case "time and samples" --oracle-differs "$LATE_LIMIT" --no-output --filter '2,$d' --tx-filter "$N" --timeout 20 -- -d $K --time 300ms --samples 100
    emu_case "time 1 ms" --oracle-differs "$LATE_LIMIT" --tx-filter "$N" --no-output --timeout 20 -- -d $K --time 1ms
    emu_case "time warns of no group" --oracle-differs "$LATE_LIMIT" --stderr --tx-filter "$N" --timeout 20 -- -d $K --time 300ms
    emu_options

    # Silence, where C has no end: protocol.c:193 returns before any limit.
    emu_options period=5 scan-burst=2 burst=1 frame=$TI_BASE,$TI_BASE,$TI_BASE
    emu_case "lib silence and samples" --libsigrok --hangs --timeout 3 -- -d $K --samples 100
    emu_case "lib silence and time" --libsigrok --hangs --timeout 3 -- -d $K --time 200ms
    emu_case "silence and time" --oracle-bug "$TIME_HANG" --oracle-hangs --timeout 5 -- -d $K --time 1s
    emu_case "unplug in a capture" --oxrok-fails "the port gone" --oracle-bug "$HUP" --oracle-hangs --timeout 5 --unplug model -- -d $K --samples 100
    emu_case "interrupt a silent meter" --oracle-bug "$SIGINT_BUG" --timeout 5 --interrupt model -- -d $K --samples 100
    emu_options

    # A meter silent from the start: 15 s for C to show it hangs and 10 s for oxrok's
    # bound, the one case that shows that bound. After a sample it warns and goes on,
    # for minutes, which the unit tests hold.
    emu_options period=5 scan-burst=2 burst=0 frame=$TI_BASE,$TI_BASE
    emu_case "silence and samples" --oracle-bug "$NO_BOUND" --oracle-hangs --oxrok-fails "no bytes from the meter for 10 s" --timeout 15 --no-output -- -d $K --samples 100
    emu_options

    # The cable.
    emu_options period=5 scan-burst=2 stall=lines
    emu_case "stall lines" --keep-header --no-output --stderr -- -d $K --scan
    emu_options period=5 scan-burst=2 stall=line
    emu_case "stall line" --keep-header --no-output -- -d $K --scan
    emu_options
}
