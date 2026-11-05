# A UNI-T UT181A meter (emu/src/models/uni-t-ut181a.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
#
# PAD: the model's pad= takes C's parser past the byte consume_str writes over
# (protocol.c:1720, see the model), so most cases with anything behind a unit
# have it. A few cases say what the oracle's build does without it.
EMU_MODEL=uni-t-ut181a
EMU_CDC=1
EMU_DRIVER=uni-t-ut181a
EMU_FULL=$EMU_DRIVER
EMU_GCOV=uni-t-ut181a
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    PAD=pad=1500
    # oxrok's log prefix, on its side only, and its errors beside the names
    # sr_strerror() has for them; a refused start's last lines, each tool's own.
    F='s/^oxrok: [a-z]+: oxrok::drivers::uni_t_ut181a(::protocol)?: /sr: uni-t-ut181a: /
s/^(Failed to (get|set device option) .[a-z_]+.: )not supported: [a-z_]+/\1not applicable/
s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\1./
s/^Failed to set channels: invalid argument: //
/^sr: session: Could not start/d
s/^(Failed to start session\.|timeout)$/start refused/'
    FIRST_BUG="ut181a_query_initial_state returns at the first 100 ms of ut181a_waitfor_response (api.c:140-142), so its 2 s deadline (api.c:145) never counts; oxrok waits the 2 s"
    STACK_BUG="consume_str writes a NUL at buff[sz] (protocol.c:1720) of an 8 byte unit_buff (protocol.c:1740); in this -O0 build that is the low byte of pl_dlen (DWARF: unit_buff at -144, pl_dlen at -136), so what is behind a unit reads as cut, and a record info as no recording; an -O2 build has other bytes there"
    UTF8_BUG="g_variant_new_strv on a name that is no UTF-8 (api.c:447) is a critical, and sigrok-cli's logger exits on it; oxrok shows U+FFFD"
    MQ_BUG="config_set passes sdi->conn as the sdi of ut181a_waitfor_response (api.c:412), whose priv is then no dev_context: it crashes; oxrok waits for the reply code"
    AUTO_BUG="range=auto calls ut181a_send_cmd_setmode with a range of 0 (protocol.c:684), so the meter gets a mode of 0 and the range stays; oxrok sends a range of 0, as Protocol.md has it"
    CACHE="C's range reads the last measurement's byte, so it says auto until the next one (protocol.c:510-531); oxrok says what the meter took"
    TIME_BUG="a time limit is checked only at a packet (protocol.c:2643, and the send_feed's check), so C goes on to the next reading; oxrok looks on each pass of its loop"
    NO_TIME_BUG="a time limit is checked only at a packet (protocol.c:2643), and a silent meter has none, so it never ends; oxrok does"
    ER_BUG="an ER moves nothing on (protocol.c:1849-1852), so a download waits for ever; oxrok ends it"
    SILENT_BUG="a request with no answer is waited for for ever: nothing else sends the next (protocol.c:2656-2689); oxrok ends after 5 s with nothing read"
    EMPTY_BUG="a chunk of no samples asks again for the same ones (protocol.c:2677-2682), for ever; oxrok ends there, as another host of the meter does"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    CHUNK_BUG="the next chunk asks for rec_data.rec_idx (protocol.c:2682), which nothing sets, so it comes from recording 1; oxrok asks for the one selected"
    # A scan's reading, ahead of a list: the scan takes the first of a list.
    S=na@3111.1:m=5/30/VDC

    # The scan: no id to ask for, so a port that answers is the meter.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "no conn" --keep-header --no-output -- -d uni-t-ut181a --scan
    emu_case "no such node" --keep-header --no-output -- -d uni-t-ut181a:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm misset" --keep-header --no-output -- -d $U:serialcomm=19200/8n1 --scan
    emu_case "serialcomm no such" --tx-filter '/^ctl_out/d' --keep-header --no-output -- -d $U:serialcomm=fast --scan
    emu_options setters=0 uart=19200/8n1
    emu_case "serialcomm on a changed line" --keep-header -- -d $U:serialcomm=19200/8n1 --scan
    # What goes wrong in a scan, step by step.
    emu_options setters=0 fail=05:1:silent
    emu_case "scan no reading" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 fail=05:1:late:20
    emu_case "scan first reading late" --oracle-differs "$FIRST_BUG" --keep-header -- -d $U --scan
    emu_options setters=0 fail=05:2:silent
    emu_case "scan monitor off silent" --keep-header -- -d $U --scan
    emu_options setters=0 fail=0e:1:silent
    emu_case "scan no count" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 fail=0e:1:er
    emu_case "scan count refused" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 fail=0e:1:bad
    emu_case "scan count checksum" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 pad=300 rec=A/VDC/1/1 fail=0c:1:silent
    emu_case "scan no record info" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 pad=300 rec=A/VDC/1/1,B/VDC/1/1 fail=0c:2:er
    emu_case "scan second record refused" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 pad=300 rec=_/VDC/1/1
    emu_case "scan record with no name" --keep-header --no-output -- -d $U --scan
    # What the oracle's build does with any recording, without the padding.
    emu_options setters=0 rec=A/VDC/1/1
    emu_case "scan with a recording" --oracle-differs "$STACK_BUG" --keep-header -- -d $U --scan
    # A first reading C cannot use.
    emu_options setters=0 readings=n@1234.1:m=5/30/VDC
    emu_case "scan mode unknown" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 readings=n@0.1:m=5/30/VDC
    emu_case "scan mode 0" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=n@1234.1:m=5/30/VDC,$S
    emu_case "scan a second reading after a bad one" --keep-header -- -d $U --scan
    # What comes ahead of a frame: the search for the next magic.
    emu_options setters=0 first=2 period=2 readings=raw:00112233445566,$S
    emu_case "scan garbage ahead" --keep-header -- -d $U --scan
    emu_options setters=0 readings=frame:02
    emu_case "scan frame too short" --keep-header --no-output -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=raw:abcd0100ffff,$S
    emu_case "scan length too small" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=raw:abcd0010ffff,$S
    emu_case "scan length too large" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=raw:abcd0300020000,$S
    emu_case "scan checksum wrong" --keep-header -- -d $U --scan
    # All frames C handles or refuses without a scan's wait being met.
    emu_options setters=0 first=2 period=2 readings=frame:,$S
    emu_case "scan empty frame" --stderr --filter "$F" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:77,$S
    emu_case "scan unknown kind" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:01,$S
    emu_case "scan short reply code" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:72,$S
    emu_case "scan short reply data" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:0500,$S
    emu_case "scan stray samples" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:03,$S
    emu_case "scan stray save short" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:02,$S
    emu_case "scan measurement with no head" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:0200,$S
    emu_case "scan measurement cut after misc" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:020000,$S
    emu_case "scan measurement cut after misc2" --keep-header -- -d $U --scan
    emu_options setters=0 first=2 period=2 readings=frame:0200001131,$S
    emu_case "scan measurement cut before the range" --keep-header -- -d $U --scan

    # --show and what it asks again.
    emu_options setters=0
    emu_case "show" --keep-header -- -d $U --show
    emu_options setters=0 pad=300 rec=_Lead_/VDC/1/1,Two/mVAC/5/2,LONGESTNAME/VDC/1/1
    emu_case "show recordings" --keep-header -- -d $U --show
    emu_options setters=0 pad=300 recs=25
    emu_case "show more recordings than it holds" --keep-header -- -d $U --show
    emu_options setters=0 fail=0e:2:silent
    emu_case "show count silent the second time" --keep-header -- -d $U --show
    emu_options setters=0 pad=300 rec=R^1/VDC/1/1
    emu_case "show a name no UTF-8" --oracle-bug "$UTF8_BUG" --keep-header -- -d $U --show

    # --get: a key each, and range's tables, from the scan's last reading.
    emu_options setters=0
    for key in conn limit_samples limit_frames limit_time data_source datalog measured_quantity range; do
        emu_case "get $key" -- -d $U --get $key
    done
    emu_options setters=0 readings=nR@3111.1:m=5/30/VDC
    emu_case "get datalog on" -- -d $U --get datalog
    for r in 3111.1 3111.2 3111.3 3111.4 2111.1 2111.2 5111.6 5211.1 5311.1 6211.8 7111.7 7211.4 8111.2 8211.1 9111.2 9211.1 4211.1 4311.1 6111.1 a111.1 ; do
        emu_options setters=0 readings=n@$r:m=5/30/VDC
        emu_case "get range $r" -- -d $U --get range
        emu_case "get mq $r" -- -d $U --get measured_quantity
    done
    emu_options setters=0 readings=n@3111.0:m=5/30/VDC
    emu_case "get range 0" -- -d $U --get range
    emu_options setters=0 readings=n@3111.9:m=5/30/VDC
    emu_case "get range above all" --stderr --filter "$F" --expect-fail -- -d $U --get range
    emu_options setters=0 readings=n@3111.5:m=5/30/VDC
    emu_case "get range past the table" --stderr --filter "$F" --expect-fail -- -d $U --get range

    # -c: what is only stored, what goes out, what is refused.
    emu_options setters=0
    emu_case "set limit_samples" -- -d $U -c limit_samples=5 --get limit_samples
    emu_case "set limit_frames" -- -d $U -c limit_frames=5 --get limit_frames
    emu_case "set limit_time" -- -d $U -c limit_time=5 --get limit_time
    emu_case "set datalog" --stderr --filter "$F" --expect-fail -- -d $U -c datalog=on --set
    emu_options setters=0 pad=300 rec=Two/VDC/1/1,Two/VDC/1/1,Three/VDC/1/1
    emu_case "set data_source Save" -- -d $U -c data_source=Save --get data_source
    emu_case "set data_source Live" -- -d $U -c data_source=Live --get data_source
    emu_case "set data_source a name" -- -d $U -c data_source=Three --get data_source
    emu_case "set data_source a number" -- -d $U -c data_source=3 --get data_source
    emu_case "set data_source a spaced number" -- -d $U -c "data_source=2 " --get data_source
    emu_case "set data_source 0" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=0 --set
    emu_case "set data_source too large" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=4 --set
    emu_case "set data_source no such" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=Nothing --set
    # The range lists, per kind of mode: what goes out, and what is refused.
    for s in 3111:6V 3111:1000V 2111:600mV 6211:600mF 7111:60MHz 7211:60kHz 8111:6000uA 9211:60mA; do
        emu_options readings=n@${s%%:*}.1:m=5/30/VDC
        emu_case "set range ${s#*:} at ${s%%:*}" --no-output -- -d $U -c range=${s#*:} --set
    done
    # GOption takes this text only as GLib finds the charset, which sigrok-cli never
    # asks of the locale.
    emu_options readings=n@5111.1:m=5/30/VDC
    emu_case "set range 60MOhm at 5111" --oracle-env CHARSET=UTF-8 --no-output -- -d $U -c range=60MΩ --set
    for m in 5211 5311 4211 4311 6111 a111; do
        emu_options setters=0 readings=n@$m.1:m=5/30/VDC
        emu_case "set range refused at $m" --stderr --filter "$F" --expect-fail -- -d $U -c range=6V --set
    done
    emu_options setters=0
    emu_case "set range not of the mode" --stderr --filter "$F" --expect-fail -- -d $U -c range=60mA --set
    emu_case "set range no such" --stderr --filter "$F" --expect-fail -- -d $U -c range=7V --set
    emu_options
    emu_case "set range then get" --oracle-differs "$CACHE" -- -d $U -c range=6V --get range
    emu_case "set range auto" --oracle-differs "$AUTO_BUG" --no-output -- -d $U -c range=auto --set
    emu_options setters=0
    emu_case "set range empty" --stderr --filter "$F" --expect-fail -- -d $U -c range= --set
    # measured_quantity: what the table has.
    emu_options setters=0
    emu_case "set mq not in the table" --stderr --filter "$F" --expect-fail -- -d $U -c measured_quantity=power --set
    emu_options
    emu_case "set mq" --oracle-bug "$MQ_BUG" --oracle-env GLIBC_TUNABLES=glibc.malloc.perturb=165 --no-output -- -d $U -c measured_quantity=voltage/ac --set

    # A live capture: one reading per format, each as far as it goes.
    live() {
        emu_options setters=0 period=2 $PAD frames=${2:-1} readings=$S,$1
    }
    emu_options setters=0
    emu_case "samples 1" -- -d $U --samples 1 -O analog
    emu_options setters=0 period=2 frames=2
    emu_case "samples 2" -- -d $U --samples 2 -O analog
    emu_options setters=0 period=2 frames=2
    emu_case "frames 2" -- -d $U --frames 2 -O analog
    # A stream that goes on: the reading after the stop's one is far off.
    emu_options setters=0 period=100
    emu_case "samples 1 of a stream" -- -d $U --samples 1 -O analog
    live "n@3111.1:m=1/30/VDC:1=2/21/Hz"
    emu_case "aux1" -- -d $U --samples 1 -O analog
    emu_case "aux1 with frames" -- -d $U --frames 1 -O analog
    live "n@3111.1:m=1/30/VDC:2=3/12/dBV"
    emu_case "aux2 alone" -- -d $U --frames 1 -O analog
    live "n@3111.1:m=1/30/VDC:1=2/30/VDC:2=3/30/VDC:b=4/VDC"
    emu_case "aux and bar" --keep-header -- -d $U --frames 1 -O analog
    live "nh@3111.1:m=1/30/VDC:b=4/VDC"
    emu_case "hold and bar" -- -d $U --frames 1 -O analog
    # Without the padding: the first field behind a unit fails, with the frame
    # open, in the oracle's build.
    emu_options setters=0 period=25 frames=3 "readings=$S,n@3111.1:m=1/30/VDC:1=2/21/Hz,n@3111.1:m=7/30/VDC,n@3111.1:m=8/30/VDC"
    emu_case "aux in the oracle's build" --oracle-differs "$STACK_BUG" --keep-header -- -d $U --samples 2 -O analog
    # COMP, its four forms: the limits go nowhere.
    C='s/^sr: \[[0-9:.]*\] /sr: /
/uni-t-ut181a: Unprocessed/!d'
    live "nc@3111.1:m=1/30/VDC:c=0/0/3/3.3/0"
    emu_case "comp inner" --stderr --filter "$F
$C" -- -d $U --frames 1 -O analog -l 4
    live "nc@3111.1:m=1/30/VDC:c=1/1/3/3.3/1"
    emu_case "comp outer" --stderr --filter "$F
$C" -- -d $U --frames 1 -O analog -l 4
    live "nc@3111.1:m=1/30/VDC:c=2/0/3/30"
    emu_case "comp below" --stderr --filter "$F
$C" -- -d $U --frames 1 -O analog -l 4
    live "nc@3111.1:m=1/30/VDC:c=3/1/3/30"
    emu_case "comp above" --stderr --filter "$F
$C" -- -d $U --frames 1 -O analog -l 4
    emu_options setters=0 period=25 frames=3 "readings=$S,nc+600@3111.1:m=1/30/VDC:c=4/1/3/30,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "comp no such mode" --keep-header -- -d $U --samples 2 -O analog
    # The other formats.
    live "r@3112.1:m=1/30/VDC:1=2/30/VDC:2=3/30/VDC:b=4/VDC"
    emu_case "relative" -- -d $U --frames 1 -O analog
    live "x@3111.1:1/30:2/30/7:3/30/8:4/30/9:VDC"
    emu_case "minmax" -- -d $U --frames 1 -O analog
    live "p@3131.1:2/30/VDC:1/30/VDC"
    emu_case "peak" -- -d $U --frames 1 -O analog
    emu_case "samples at a peak" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,t3@3111.1,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "format 3 leaves its frame open" --keep-header -- -d $U --samples 1 -O analog
    # Flags that change nothing.
    live "nlv@3111.1:m=1/30/VDC"
    emu_case "lead error and high voltage" -- -d $U --frames 1 -O analog
    # Each field cut, in frames still good. C makes one step of the search per
    # read that brings bytes, so a frame behind a failed one waits for the next,
    # and the two must not come in one read: hence the period.
    emu_options setters=0 period=25 frames=3 "readings=$S,n~13@3111.1:m=1/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "main cut at its value" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,n~9@3111.1:m=1/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "main cut at its precision" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,n~8@3111.1:m=1/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "main cut at its unit" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,r~8@3112.1:m=1/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "relative cut at its unit" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,x~8@3111.1:1/30:2/30/7:3/30/8:4/30/9:VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "minmax cut at its unit" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,p~21@3131.1:2/30/VDC:1/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "peak cut at its unit" --keep-header -- -d $U --samples 1 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,n~1008+1000@3111.1:m=1/30/VDC:1=2/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "aux1 cut at its unit" --keep-header -- -d $U --samples 2 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,n~1008+1000@3111.1:m=1/30/VDC:b=4/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "bar cut at its unit" --keep-header -- -d $U --samples 2 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,nc~1004+1000@3111.1:m=1/30/VDC:c=0/0/3/3.3/0,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "comp cut at its low limit" --keep-header -- -d $U --samples 2 -O analog
    emu_options setters=0 period=25 frames=3 "readings=$S,r~1008+1000@3112.1:m=1/30/VDC:1=2/30/VDC,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "reference cut at its unit" --keep-header -- -d $U --samples 2 -O analog
    # Overloads, and digits.
    live "n@3111.1:m=1/31/VDC,n@3111.1:m=1/32/VDC,n@3111.1:m=1/33/VDC,n@3111.1:m=1.23456/f0/VDC,n@3111.1:m=-1.5/00/VDC" 5
    emu_case "overloads and digits" -- -d $U --samples 5 -O analog
    # Every unit text, with its prefixes.
    UNITS=
    n=0
    for u in VDC VAC Vac+dc mVDC mVAC mVac+dc ADC AAC Aac+dc mADC uAAC nF uF mF pF dBV dBm '~' 'k~' 'M~' 'G~' nS % ms s Hz kHz MHz ^C ^F x ; do
        UNITS="$UNITS,n@3111.1:m=1234.5/20/$u"
        n=$((n+1))
    done
    emu_options setters=0 period=2 frames=$n readings=$S$UNITS
    emu_case "every unit" -- -d $U --samples $n -O analog
    emu_options setters=0 period=2 frames=3 "readings=$S,n@3111.1:m=1/30/_VDC_,n@3111.1:m=1/30/MMMMMMMM,n@3111.1:m=1/30/"
    emu_case "unit forms" -- -d $U --samples 3 -O analog
    # MQ flags from the mode and the unit.
    emu_options setters=0 period=2 frames=4 "readings=$S,na@6111.1:m=0.5/30/VDC,nh@3111.1:m=1/30/VDC,na@5111.1:m=1/30/k~,nah@3121.1:m=1/30/Vac+dc"
    emu_case "mode flags" -- -d $U --samples 4 -O analog
    # Unknown frames in a capture go by without a sample.
    emu_options setters=0 period=25 frames=5 "readings=$S,frame:77,n@1234.1:m=1/30/VDC,raw:0011,n@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC"
    emu_case "bad frames between" --keep-header -- -d $U --samples 1 -O analog
    # A byte a transfer.
    emu_options setters=0 period=2 frames=1 chunk=1
    emu_case "samples one byte per transfer" -- -d $U --samples 1 -O analog
    # The limits and channels.
    emu_options setters=0 period=2 frames=1
    emu_case "channel P1" -- -d $U -C P1 --samples 1 -O analog
    live "n@3111.1:m=1/30/VDC:1=2/21/Hz"
    emu_case "channel P2 alone" -- -d $U -C P2 --frames 1 -O analog
    emu_options setters=0 period=2 frames=1
    emu_case "no such channel" --stderr --filter "$F" --expect-fail -- -d $U -C P9 --samples 1 -O analog
    emu_case "limit_samples 0" --stderr --filter "$F" --expect-fail -- -d $U -c limit_samples=0 --samples 1
    emu_options setters=0 period=100
    emu_case "time" --oracle-differs "$TIME_BUG" -- -d $U --time 400 -O analog
    emu_options setters=0 period=2 frames=0
    emu_case "time on a silent meter" --oracle-bug "$NO_TIME_BUG" --oracle-hangs --timeout 3 --no-output -- -d $U --time 100 -O analog
    emu_case "samples on a silent meter" --libsigrok --hangs --timeout 2 --no-output -- -d $U --samples 1 -O analog
    # sigrok-cli stops a --continuous at stdin's end, here at once, and oxrok at the
    # time limit. With a reading to read, whether C has read it would be a race.
    emu_options setters=0 period=100 frames=0
    emu_case "continuous and a time limit" --no-output -- -d $U --continuous --time 200 -O analog
    # A library client, whose stop comes from the main loop.
    emu_options setters=0 period=2 frames=3
    emu_case "lib stop" --libsigrok -- -d $U --continuous --stop-after 2 -O analog
    emu_case "lib continuous stops after the stream" --libsigrok --hangs --timeout 3 -- -d $U --continuous -O analog
    # The monitor off, unanswered or with a reading behind it.
    emu_options setters=0 period=2 frames=1 fail=05:4:silent
    emu_case "monitor off silent" -- -d $U --samples 1 -O analog

    # Saved readings, one request after each answer.
    emu_options setters=0 $PAD saved=na@3111.1:m=1.5/30/VDC,n@2111.2:m=2/20/mVAC:1=3/10/Hz,p@3131.1:4/30/VDC:3/30/VDC,r@3112.1:m=1/30/VDC:1=2/30/VDC:2=3/30/VDC,x@3111.1:1/30:2/30/7:3/30/8:4/30/9:VDC
    emu_case "save" -- -d $U -c data_source=Save --samples 100 -O analog
    emu_case "save samples 2" -- -d $U -c data_source=Save --samples 2 -O analog
    emu_case "save frames 3" -- -d $U -c data_source=Save --frames 3 -O analog
    emu_case "save time" -- -d $U -c data_source=Save --time 10000 -O analog
    emu_options setters=0 $PAD saved=na@3111.1:m=1.5/30/VDC,n~1600@3111.1:m=2/30/VDC,n@3111.1:m=3/30/VDC
    emu_case "save a frame cut holds it" --libsigrok --hangs --timeout 2 -- -d $U -c data_source=Save --samples 100 -O analog
    emu_options setters=0
    emu_case "save none" --oracle-bug "$ER_BUG" --oracle-hangs --timeout 3 --no-output -- -d $U -c data_source=Save --samples 1 -O analog
    emu_options setters=0 fail=08:1:silent saved=n@3111.1:m=1/30/VDC
    emu_case "save no count" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=Save --samples 1 -O analog
    emu_options setters=0 fail=08:1:er saved=n@3111.1:m=1/30/VDC
    emu_case "save count refused" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=Save --samples 1 -O analog
    emu_options setters=0 fail=07:2:silent saved=n@3111.1:m=1/30/VDC,n@3111.1:m=2/30/VDC
    emu_case "save second silent" --oracle-bug "$SILENT_BUG" --oracle-hangs --timeout 7 -- -d $U -c data_source=Save --samples 5 -O analog
    emu_options setters=0 fail=07:2:er saved=n@3111.1:m=1/30/VDC,n@3111.1:m=2/30/VDC
    emu_case "save second refused" --libsigrok --hangs --timeout 2 -- -d $U -c data_source=Save --samples 5 -O analog
    emu_options setters=0 saved=n@1234.1:m=1/30/VDC,n@3111.1:m=2/30/VDC
    emu_case "save a mode unknown holds it" --libsigrok --hangs --timeout 2 --no-output -- -d $U -c data_source=Save --samples 5 -O analog

    # Recordings: an info, then chunks.
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/9,Two/mVAC/5/2,Three/kHz/1/5,Empty/VDC/1/0 over=3:13
    emu_case "recording first" -- -d $U -c data_source=One --samples 100 -O analog
    emu_case "recording by number" -- -d $U -c data_source=2 --samples 100 -O analog
    emu_case "recording second in chunks" --oracle-differs "$CHUNK_BUG" -- -d $U -c data_source=Three --samples 100 -O analog
    emu_case "recording empty" --no-output -- -d $U -c data_source=Empty --samples 100 -O analog
    emu_case "recording samples 2" -- -d $U -c data_source=One --samples 2 -O analog
    emu_case "recording samples within a chunk" -- -d $U -c data_source=One --samples 1 -O analog
    emu_case "recording frames 3" -- -d $U -c data_source=One --frames 3 -O analog
    emu_case "recording time" -- -d $U -c data_source=One --time 10000 -O analog
    emu_options setters=0 pad=300 per=255 recs=1
    emu_case "recording one chunk" -- -d $U -c data_source=REC1 --samples 100 -O analog
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/3 fail=0c:2:silent
    emu_case "recording info silent" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=One --samples 100 -O analog
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/3 fail=0c:2:er
    emu_case "recording info refused" --stderr --filter "$F" --expect-fail -- -d $U -c data_source=One --samples 100 -O analog
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/3 fail=0d:2:silent
    emu_case "recording chunk silent" --libsigrok --hangs --timeout 2 -- -d $U -c data_source=One --samples 9 -O analog
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/3 fail=0d:2:er
    emu_case "recording chunk refused" --libsigrok --hangs --timeout 2 -- -d $U -c data_source=One --samples 9 -O analog
    # A device that says it has more than it gives: C asks again for ever.
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/5 have=3
    emu_case "recording shorter than its info" --oracle-bug "$EMPTY_BUG" --oracle-hangs --timeout 3 -- -d $U -c data_source=One --samples 100 -O analog
    emu_options setters=0 pad=300 per=2 rec=One/VDC/2/3 fail=0d:1:bad
    emu_case "recording chunk checksum" --libsigrok --hangs --timeout 2 --no-output -- -d $U -c data_source=One --samples 9 -O analog

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $U --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $U --scan
    emu_options setters=0 fail=0e:1:silent
    emu_case "unplug in the scan" --unplug model --timeout 30 --keep-header --no-output -- -d $U --scan
    # An interrupt, once the stream has run out: only that can end this capture,
    # and the monitor off still goes.
    emu_options setters=0 period=2 frames=1
    emu_case "interrupt a capture" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 8 -- -d $U --continuous -O analog
    emu_options setters=0 period=2 frames=1
    emu_case "unplug in a capture" --stderr --filter "$F
/^sr: serial-libsp: Write error/d" --expect-fail --unplug model --timeout 10 -- -d $U --samples 5 -O analog
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    U=uni-t-ut181a:conn=$EMU_TTY
}
