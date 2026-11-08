# Yokogawa DL and DLM scopes (emu/src/models/yokogawa-dlm.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=yokogawa-dlm
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=yokogawa-dlm
EMU_FULL=$EMU_DRIVER
EMU_GCOV=yokogawa-dlm
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

# yokogawa-dlm/api.c's start sends no header, and sigrok-cli skips all before a header
# (session.c:286), so a stock capture prints nothing and exits 0: the "stock" cases pin
# that. Every other capture case below runs sigrok-cli under tools/oracle-dlm-header.c,
# which gives its callback the missing header ahead of the first packet, and changes
# nothing else: the driver is stock, and its wire and its packets are compared. Its
# labels are said to hold on stock C where they do; the others say "under the preload".
dlm_header() {
    DLM_STOCK_CLI=$SIGROK_CLI
    [ "$no_oracle" = 1 ] && return
    cc -shared -fPIC -o "$TMP/dlm-header.so" "$TOOLS/oracle-dlm-header.c" -ldl 2>"$TMP/dlm-header.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    printf '%s\n' '#!/bin/sh' \
        "LD_PRELOAD=$TMP/dlm-header.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$SIGROK_CLI") \"\$@\"" \
        >"$TMP/dlm-sigrok-cli"
    chmod +x "$TMP/dlm-sigrok-cli"
    DLM_HEADER_CLI=$TMP/dlm-sigrok-cli
}

emu_cases() {
    dlm_header
    A='1~64!d'
    U=yokogawa-dlm
    T=yokogawa-dlm:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    F="--frames 1"
    # oxrok's log, in sigrok's prefix.
    L='s/^oxrok: err: oxrok::drivers::yokogawa_dlm: /sr: yokogawa-dlm: /
s/^oxrok: warn: oxrok::drivers::yokogawa_dlm: /sr: yokogawa-dlm: /
s/^Failed to open device: .*/Failed to open device./'
    # A refused start in the core's words, on C's side only: oxrok's stay.
    S="$L
s/^sr: session: yokogawa-dlm device \\(null\\) has no enabled channels\\.\$/No channels enabled./
/^sr: session: Could not start /d
/^sr: session: Cannot remove /d
/^Failed to start session\\.\$/d"

    # And C's last line as the words oxrok ends with.
    S2="$L
/^sr: session: Could not start /d
/^sr: session: Cannot remove /d
s/^Failed to start session\\.\$/not supported: Invalid channel configuration specified!/"

    NO_HEADER="api.c's dev_acquisition_start never calls std_session_send_df_header and sigrok-cli's datafeed_in skips every packet before a header (session.c:286): a stock capture prints nothing and exits 0; oxrok sends one"
    NO_PODS="dlm_device_init allocates g_malloc0(0) for a model with no pods, which is NULL, and takes that for no memory (protocol.c:780-786), so stock C finds no such scope; oxrok does"
    MISREAD="array_float_get takes three bytes of an exponent, 0 of none, and drops its sign (protocol.c:337, :345): C reads another entry of the table with exit 0, and oxrok refuses the open"
    RESULT_AND="dlm_analog_chan_wrange_get and _woffs_get say result &= get_float, and SR_OK is 0, so any failure is OK (protocol_wrappers.c:197, :209): C scales by a range or offset of 0 with exit 0, and oxrok fails the open"
    LENGTH="sr_atol takes an empty answer for 0 and -5 wraps into a uint32_t: C reads a frame of no samples or says Truncated, with exit 0, and oxrok fails the open"
    COUNT="dlm_analog_samples_send reads samples_per_frame bytes and never the header's count (protocol.c:919): C takes a line end for a sample, or a block short of what it says, with exit 0; oxrok says it is cut short"
    UAF="on an empty block protocol.c:1074 frees the static buffer and keeps the pointer, and the next read writes to it (under the preload; stock too, which has only no output to show): sigrok-cli crashes, and oxrok stops the scope and asks again"
    INDEX="logic channels have the index 32 + bit (DLM_DIG_CHAN_INDEX_OFFS), and the bits output reads the byte at index / 8 of a one byte sample: C shows each channel 4 samples on, and bytes past the buffer at the end (under the preload); oxrok shows the bit of its own sample"
    NAN_="config_set's two comparisons pass a NaN (api.c), and :TRIGGER:DELAY:TIME -NAN goes out with exit 0; oxrok refuses it"
    WARNS="C ignores frame_limit with no word; oxrok warns that it reads one frame"
    WHOLE="an answer of exactly a read's size has no short read to end it, and the recv blocks with no timeout (scpi_tcp.c, tcp.c), so C never ends (under the preload, and stock); oxrok ends on a read's silence"
    SIGPIPE="sr_tcp_write_bytes is a plain send() and nothing ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli; oxrok's write returns EPIPE and the capture fails"
    NO_SIGINT="sigrok-cli has no SIGINT handler; oxrok stops its capture and exits 0"
    STUCK='/^tcp_out 3a53544f50/{x;/F/{x;d;};s/$/F/;x;}
/^tcp_in 11 -> 233830303030303030300a$/{x;/G/{x;d;};s/$/G/;x;}'

    # Scans, by every name in the table.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_reset
    emu_case "usbtmc by address" --keep-header -- -d yokogawa-dlm:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header -- -d yokogawa-dlm:conn=usbtmc --scan
    emu_case "usbtmc kernel driver bound" --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' --keep-header -- -d $U --scan
    emu_case "tcp nothing listens" --no-output -- -d yokogawa-dlm:conn=tcp-raw/127.0.0.1/1 --scan
    for m in DLM2034 DLM2054 DL9505L DL9510L DL9705L DL9710L; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
    done
    # With no pods, g_malloc0(0) is NULL, and dlm_device_init takes that for no memory.
    for m in DLM2022 DLM2032 DLM2052 DL9040 DL9040L DL9140 DL9140L DL9240 DL9240L; do
        emu_options model=$m
        emu_case "scan $m" --keep-header --oracle-differs "$NO_PODS" -- -d $T --scan
    done
    emu_options idn=Acme,710110,1,2
    emu_case "scan another maker" --no-output -- -d $T --scan
    emu_options idn=YOKOGAWA,710140,1,2
    emu_case "scan another model" --stderr --filter "$L" -- -d $T --scan
    emu_options idn=short
    emu_case "scan no idn to speak of" --no-output -- -d $U --scan
    emu_options idn=YOKOGAWA,710110,F1.00
    emu_case "scan three fields" --keep-header -- -d $T --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options late-at=1:5
    emu_case "scan late" --keep-header -- -d $U --scan
    emu_options

    # Show: what every row has.
    emu_case "show" --keep-header -- -d $T --show
    emu_case "show usb" --keep-header -- -d $U --show
    emu_case "show group 1" -- -d $T -g 1 --show
    emu_case "show pod" -- -d $T -g POD0 --show
    for m in DL9505L DL9705L; do
        emu_options model=$m
        emu_case "show $m" --keep-header -- -d $T --show
    done
    for m in DLM2022 DL9040; do
        emu_options model=$m
        emu_case "show $m" --keep-header --oracle-bug "$NO_PODS" -- -d $T --show
    done
    emu_options model=DL9505L
    emu_case "show pod 3 of DL9505L" -- -d $T -g POD3 --show
    emu_options on=2,D1 logic=1 say=TRIGGER:DELAY:TIME?=2.00E-03 say=TRIGGER:ATRIGGER:SIMPLE:SOURCE?=BIT3 say=TRIGGER:ATRIGGER:SIMPLE:SLOPE?=FALL say=TIMEBASE:TDIV?=2.000E-04
    emu_case "show another state" --keep-header -- -d $T --show
    emu_options

    # Gets.
    emu_case "get samplerate" -- -d $T --get samplerate
    emu_case "get timebase" -- -d $T --get timebase
    emu_case "get num_hdiv" -- -d $T --get num_hdiv
    emu_case "get horiz_triggerpos" -- -d $T --get horiz_triggerpos
    emu_case "get triggersource" -- -d $T --get triggersource
    emu_case "get triggerslope" -- -d $T --get triggerslope
    emu_case "get vdiv" -- -d $T -g 2 --get vdiv
    emu_case "get vdiv no group" --expect-fail --stderr -- -d $T --get vdiv
    emu_case "get coupling" -- -d $T -g 2 --get coupling
    emu_case "get num_vdiv" -- -d $T -g 2 --get num_vdiv
    emu_case "get limit frames" --expect-fail --stderr --filter 's/^(Failed to get .limit_frames.: ).*/\1not applicable/' -- -d $T --get limit_frames

    # Settings. A table's whole, through %E.
    TB=""; for v in 1ns 2ns 5ns 10ns 20ns 50ns 100ns 200ns 500ns 1us 2us 5us 10us 20us 50us 100us 200us 500us 1ms 2ms 5ms 10ms 20ms 50ms 100ms 200ms 500ms 1s 2s 5s 10s 20s 50s 100s 200s 500s; do TB="$TB -c timebase=$v"; done
    VD=""; for v in 2mV 5mV 10mV 20mV 50mV 100mV 200mV 500mV 1V 2V 5V 10V 20V 50V 100V 200V 500V; do VD="$VD -c vdiv=$v"; done
    emu_case "set each timebase" --no-output -- -d $T $TB --set
    emu_case "set each vdiv" --no-output -- -d $T -g 1 $VD --set
    emu_case "set timebase usb" --no-output -- -d $U -c timebase=2ms --set
    emu_case "set a timebase with no entry" --no-output --expect-fail -- -d $T -c timebase=3ms --set
    emu_case "set a vdiv with no entry" --no-output --expect-fail -- -d $T -g 1 -c vdiv=3V --set
    emu_case "set vdiv no group" --no-output --expect-fail -- -d $T -c vdiv=1V --set
    emu_case "set vdiv on a pod" --no-output --expect-fail -- -d $T -g POD0 -c vdiv=1V --set
    for c in AC DC DC50 GND; do
        emu_case "set coupling $c" --no-output -- -d $T -g 3 -c coupling=$c --set
    done
    emu_case "set coupling of no such" --no-output --expect-fail -- -d $T -g 3 -c coupling=ICE --set
    SRC=""; for v in 1 2 3 4 LINE EXT BIT1 BIT2 BIT3 BIT4 BIT5 BIT6 BIT7 BIT8; do SRC="$SRC -c triggersource=$v"; done
    emu_case "set each trigger source" --no-output -- -d $T $SRC --set
    emu_case "set a trigger source of no such" --no-output --expect-fail -- -d $T -c triggersource=9 --set
    emu_options model=DLM2022
    emu_case "set 2ch trigger sources" --no-output --oracle-bug "$NO_PODS" -- -d $T -c triggersource=2 -c triggersource=LINE -c triggersource=EXT --set
    emu_case "set a trigger source the 2ch hasn't" --no-output --expect-fail --oracle-bug "$NO_PODS" -- -d $T -c triggersource=3 --set
    emu_options
    emu_case "set slope falling" --no-output -- -d $T -c triggerslope=f --set
    emu_case "set slope rising" --no-output -- -d $T -c triggerslope=r --set
    emu_case "set slope of no such" --no-output --expect-fail -- -d $T -c triggerslope=x --set
    emu_case "set position 0" --no-output -- -d $T -c horiz_triggerpos=0 --set
    emu_case "set position 0.25" --no-output -- -d $T -c horiz_triggerpos=0.25 --set
    emu_case "set position 0.5" --no-output -- -d $T -c horiz_triggerpos=0.5 --set
    emu_case "set position 1" --no-output -- -d $T -c horiz_triggerpos=1 --set
    emu_case "set position past 1" --no-output --expect-fail -- -d $T -c horiz_triggerpos=1.5 --set
    emu_case "set position below 0" --no-output --expect-fail -- -d $T -c horiz_triggerpos=-0.5 --set
    emu_case "set position nan" --oracle-differs "$NAN_" --expect-fail --stderr -- -d $T -c horiz_triggerpos=nan --set
    emu_options say=TIMEBASE:TDIV?=2.000E-06
    emu_case "set position at another timebase" --no-output -- -d $T -c horiz_triggerpos=0.1 --set
    emu_options
    emu_case "set limit frames" --no-output -- -d $T -c limit_frames=2 --set
    emu_options opc-late=20
    emu_case "set with opc late" --no-output -- -d $T -c triggerslope=f --set
    emu_options

    # What an open reads, wrong. Every answer's values, by the path it takes.
    for v in 1.000E-08 10.00E-09 2.000E+00 5.000E+02 1.000E+01 1.000E+02 2.900E-03; do
        emu_options say=TIMEBASE:TDIV?=$v
        emu_case "timebase answers $v" -- -d $T --get timebase
    done
    for v in abc 4.000E-03 1.000E-10 E-03 0.100E-06; do
        emu_options say=TIMEBASE:TDIV?=$v
        emu_case "timebase answers bad $v" --expect-fail --stderr --filter "$L" -- -d $T --get timebase
    done
    emu_options say=TIMEBASE:TDIV?=-2.000E-03
    emu_case "timebase answers negative" --expect-fail --stderr --filter "$L" -- -d $T --get timebase
    # What C reads as something else.
    for v in 2.000E 2.000E+03 2.000E-003; do
        emu_options say=TIMEBASE:TDIV?=$v
        emu_case "timebase misread $v" --oracle-differs "$MISREAD" --expect-fail --stderr -- -d $T --get timebase
    done
    for v in 2.000E-03 2.900E-03 5.000E+02 1.000E+00; do
        emu_options say=CHANNEL2:VDIV?=$v
        emu_case "vdiv answers $v" -- -d $T -g 2 --get vdiv
    done
    for v in abc 3.000E+00; do
        emu_options say=CHANNEL2:VDIV?=$v
        emu_case "vdiv answers bad $v" --expect-fail --stderr --filter "$L" -- -d $T -g 2 --get vdiv
    done
    emu_options say=CHANNEL2:VDIV?=2.000E+03
    emu_case "vdiv misread 2 kV" --oracle-differs "$MISREAD" --expect-fail --stderr -- -d $T -g 2 --get vdiv
    emu_options say=CHANNEL1:DISPLAY?=maybe
    emu_case "display answers maybe" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=CHANNEL1:DISPLAY?=ON
    emu_case "display answers ON" -- -d $T --get samplerate
    emu_options say=CHANNEL1:COUPLING?=ICE
    emu_case "coupling answers ICE" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=CHANNEL1:POSITION?=abc
    emu_case "position answers abc" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=LOGIC:PODA:BIT1:DISPLAY?=maybe
    emu_case "bit answers maybe" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=LOGIC:MODE?=maybe
    emu_case "logic mode answers maybe" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=TRIGGER:DELAY:TIME?=abc
    emu_case "delay answers abc" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=TRIGGER:DELAY:TIME?=5.00E-03
    emu_case "delay answers a quarter" -- -d $T --get horiz_triggerpos
    emu_options say=TRIGGER:ATRIGGER:SIMPLE:SOURCE?=9
    emu_case "source answers 9" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=TRIGGER:ATRIGGER:SIMPLE:SLOPE?=ENT
    emu_case "slope answers ENT" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=WAVEFORM:LENGTH?=abc
    emu_case "length answers abc" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=WAVEFORM:LENGTH?=12.5
    emu_case "length answers 12.5" --expect-fail --stderr --filter "$L" -- -d $T --show
    emu_options say=WAVEFORM:SRATE?=abc
    emu_case "srate answers abc" -- -d $T --get samplerate
    emu_options say=WAVEFORM:SRATE?=1.25E+09
    emu_case "srate answers 1.25G" -- -d $T --get samplerate
    emu_options say=WAVEFORM:SRATE?=1.23456789E+08
    emu_case "srate answers a float's digits" -- -d $T --get samplerate
    emu_options

    # Silence at each query an open asks: on USBTMC, as on TCP C holds for ever.
    emu_options silent-at=2
    emu_case "display silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=3
    emu_case "vdiv silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=4
    emu_case "position silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=7
    emu_case "coupling silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=26
    emu_case "bit silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=34
    emu_case "logic mode silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=35
    emu_case "timebase silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=36
    emu_case "delay silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=37
    emu_case "source silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=38
    emu_case "slope silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=39
    emu_case "length silent" --expect-fail --filter "$L" -- -d $U --show
    emu_options silent-at=40
    emu_case "srate silent" -- -d $U --get samplerate
    emu_options

    # A stock capture prints nothing.
    emu_case "stock frame prints nothing" --oracle-differs "$NO_HEADER" --digest --filter "$A" -- -d $T $F -O analog
    emu_case "stock frame usb prints nothing" --oracle-differs "$NO_HEADER" --digest --filter "$A" -- -d $U $F -O analog
    emu_case "stock binary prints nothing" --oracle-differs "$NO_HEADER" --digest --filter '1,$d' -- -d $T -C 1 $F -O binary
    # A silent :WAVEFORM:SEND?: 42 queries come ahead of it. C tries for ever, on USBTMC
    # through read_begin and on TCP in a recv with no timeout; a port's bound of 120 s is
    # a unit test, and an interrupt below.
    emu_options length=16 silent-at=42
    emu_case "lib block silent usb" --libsigrok --hangs --no-output --timeout 5 -- -d $U -C 1 $F -O analog
    emu_case "lib block silent tcp" --libsigrok --hangs --no-output --timeout 5 -- -d $T -C 1 $F -O analog
    emu_options
    # A channel's display and the pods at a stock --show.
    emu_case "stock show one channel" -- -d $T -C 1 --show
    [ "$no_oracle" = 1 ] || SIGROK_CLI=$DLM_HEADER_CLI

    # Captures: from here on under the preload.
    # Words where numbers go, through sr_atof_ascii and sr_atol.
    emu_options say=WAVEFORM:RANGE?=abc
    emu_case "range answers abc" --oracle-differs "$RESULT_AND" --expect-fail --stderr --filter "$L" -- -d $T -C 1 $F -O analog
    emu_options say=WAVEFORM:RANGE?=
    emu_case "range answers nothing" --oracle-differs "$RESULT_AND" --expect-fail --stderr --filter "$L" -- -d $T -C 1 $F -O analog
    emu_options say=WAVEFORM:OFFSET?=abc
    emu_case "offset answers abc" --oracle-differs "$RESULT_AND" --expect-fail --stderr --filter "$L" -- -d $T -C 1 $F -O analog
    emu_options say=WAVEFORM:LENGTH?=-5
    emu_case "length answers -5" --oracle-differs "$LENGTH" --expect-fail --stderr --filter "$L" -- -d $T -C 1 $F -O analog
    emu_options say=WAVEFORM:LENGTH?=
    emu_case "length answers nothing" --oracle-differs "$LENGTH" --expect-fail --stderr --filter "$L" -- -d $T -C 1 $F -O analog

    emu_options silent-at=5
    emu_case "range silent" --oracle-differs "$RESULT_AND" --expect-fail --stderr --filter "$L" -- -d $U -C 1 $F -O analog
    emu_options silent-at=6
    emu_case "offset silent" --oracle-differs "$RESULT_AND" --expect-fail --stderr --filter "$L" -- -d $U -C 1 $F -O analog
    emu_options

    emu_case "frame" --digest --filter "$A" -- -d $T $F -O analog
    emu_case "frame usb" --digest --filter "$A" -- -d $U $F -O analog
    emu_options length=16
    emu_case "frame recorded" --record -- -d $U -C 1,2 $F -O analog
    emu_options
    emu_case "frame marks" --keep-header --filter '/^[1-4]: /d' -- -d $T $F -O analog
    emu_case "two frames are one" --digest --filter "$A" -- -d $T --frames 2 -O analog
    emu_options length=16
    emu_case "two frames warn" --oracle-differs "$WARNS" --stderr --filter "$L" -- -d $T -C 1 --frames 2 -O analog
    emu_options
    emu_case "samples are no limit" --expect-fail --stderr --filter "$S" -- -d $T -C 1 --samples 10 -O analog
    emu_case "time" --expect-fail --stderr --filter "$S" -- -d $T -C 1 --time 1 -O analog
    emu_case "csv of one" --digest --filter "$A" -- -d $T -C 2 $F -O csv:header=false
    emu_case "one channel" --digest --filter "$A" -- -d $T -C 1 $F -O analog
    emu_case "channels 1 and 3" --digest --filter "$A" -- -d $T -C 1,3 $F -O analog
    emu_options on=2
    emu_case "only what is displayed" --digest --filter "$A" -- -d $T $F -O analog
    emu_case "a channel off goes on" --digest --filter "$A" -- -d $T -C 1 $F -O analog
    emu_options on=none
    emu_case "nothing displayed" --expect-fail --stderr --filter "$S" -- -d $T $F -O analog
    emu_options
    emu_case "one channel renamed" --digest --filter "$A" -- -d $T -C 1=X $F -O analog
    emu_options length=12500
    emu_case "the manual's record" --digest --filter "$A" -- -d $T -C 1 $F -O analog
    emu_case "the manual's record usb" --digest --filter "$A" -- -d $U -C 1 $F -O analog
    emu_options length=12500 message=1024
    emu_case "a message in pieces" --digest --filter "$A" -- -d $U -C 1 $F -O analog
    emu_options length=1
    emu_case "a record of one" -- -d $T -C 1 $F -O analog
    emu_options model=DLM2022
    emu_case "2ch frame" --oracle-bug "$NO_PODS" --digest --filter "$A" -- -d $T $F -O analog
    emu_options model=DL9040
    emu_case "DL9040 channel 4" --oracle-bug "$NO_PODS" --digest --filter "$A" -- -d $T -C 4 $F -O analog
    emu_options

    # Logic, a byte per sample, with bit i being Di.
    emu_options on=D0 logic=1 length=64
    emu_case "logic binary" --digest --filter '1,$d' -- -d $T $F -O binary
    emu_case "logic binary usb" --digest --filter '1,$d' -- -d $U $F -O binary
    emu_options on=D0,D1,D2,D3,D4,D5,D6,D7 logic=1 length=64
    emu_case "all logic binary" --digest --filter '1,$d' -- -d $T $F -O binary
    emu_case "all logic bits" --oracle-differs "$INDEX" --filter '1!{/^D[0-9]:/!d;}' -- -d $T $F -O bits
    emu_case "all logic hex" --oracle-differs "$INDEX" --filter '1!{/^D[0-9]:/!d;}' -- -d $T $F -O hex
    emu_options on=1,D0 logic=1 length=16
    emu_case "analog with logic" -- -d $T $F -O analog
    emu_options length=16
    emu_case "logic by -C" --digest --filter '1,$d' -- -d $T -C D0,D3 $F -O binary
    emu_case "channel 4 with logic refused" --expect-fail --stderr --filter "$S2" -- -d $T -C 4,D0 $F -O binary
    emu_case "channel 3 with logic" -- -d $T -C 3,D0 $F -O analog
    emu_options on=D1 logic=1 length=16
    emu_case "logic goes off" --digest --filter '1,$d' -- -d $T -C 1 $F -O analog
    emu_options model=DL9505L on=D0,D1 logic=1 length=16
    emu_case "DL9505L logic" --digest --filter '1,$d' -- -d $T -C A0,B7 $F -O binary
    emu_options

    # Blocks that aren't what they should be.
    emu_options length=16 cut-at=1
    emu_case "block cut short" --stderr --filter "$L" --oxrok-fails "Truncated" -- -d $T -C 1 $F -O analog
    emu_case "block cut short usb" --stderr --filter "$L" --oxrok-fails "Truncated" -- -d $U -C 1 $F -O analog
    emu_options length=16 cut-at=2
    emu_case "second block cut short" --oxrok-fails "Truncated" -- -d $T -C 1,2 $F -O analog
    emu_options length=16 claim=8
    emu_case "header says less" --oracle-differs "$COUNT" --stderr --filter "$L" --oxrok-fails "Truncated" -- -d $T -C 1 $F -O analog
    emu_options length=16 claim=32
    emu_case "header says more" --oracle-differs "$COUNT" --stderr --filter "$L" --oxrok-fails "Truncated" -- -d $T -C 1 $F -O analog
    emu_options length=16 claim=15
    emu_case "header says one less" --oracle-differs "$COUNT" --stderr --filter "$L" --oxrok-fails "Truncated" -- -d $T -C 1 $F -O analog
    emu_options length=16 say=WAVEFORM:SEND?=X
    emu_case "block is no block" --stderr --filter "$L" --oxrok-fails "malformed" -- -d $T -C 1 $F -O analog
    emu_options length=16 say=WAVEFORM:SEND?=#8abc
    emu_case "block length is no number" --stderr --filter "$L" --oxrok-fails "malformed" -- -d $T -C 1 $F -O analog
    emu_options length=16 say=WAVEFORM:SEND?=#A
    emu_case "block digit is no digit" --stderr --filter "$L" --oxrok-fails "malformed" -- -d $T -C 1 $F -O analog
    emu_options length=1 say=WAVEFORM:SEND?=#11A
    emu_case "shortest block" -- -d $T -C 1 $F -O analog
    emu_options length=16
    emu_options

    # Empty blocks: C frees the buffer it goes on using ($UAF). C crashed in each of
    # 19 runs (15 and 4, with GLIBC_TUNABLES=glibc.malloc.perturb=165), but its last
    # lines (realloc(): invalid pointer, a segmentation fault) came as the heap had it,
    # 6 of 15 runs otherwise than the first: oxrok alone, by hand-kept goldens.
    emu_options length=16 running=1
    emu_case "scope running" --no-oracle-only -- -d $T -C 1 $F -O analog
    emu_options length=16 running=stuck
    emu_case "scope stuck running" --no-output --no-oracle-only --oxrok-fails "only empty" --tx-filter "$STUCK" --timeout 25 -- -d $T -C 1 $F -O analog
    emu_options

    # An answer of exactly reads' sizes, on TCP: the short read never comes. 4096 in all.
    emu_options length=4085
    emu_case "answer of a whole read" --oracle-bug "$WHOLE" --oracle-hangs --timeout 6 --digest --filter "$A" -- -d $T -C 1 $F -O analog
    emu_options length=4085
    emu_case "answer of a whole read usb" --digest --filter "$A" -- -d $U -C 1 $F -O analog
    emu_options

    # Silence and lateness in a capture. 42 queries come ahead of the first block.
    emu_options length=16 silent-at=42 mark-at=42
    emu_case "stop while silent usb" --oracle-bug "$NO_SIGINT" --interrupt model --no-output -- -d $U -C 1 $F -O analog
    emu_case "stop while silent tcp" --oracle-bug "$NO_SIGINT" --interrupt tcp-model --no-output -- -d $T -C 1 $F -O analog
    emu_options length=16 late-at=42:10
    emu_case "block late usb" -- -d $U -C 1 $F -O analog
    emu_case "block late tcp" -- -d $T -C 1 $F -O analog
    emu_options length=16 late-at=42:100
    emu_case "block very late usb" -- -d $U -C 1 $F -O analog
    emu_case "block very late tcp" -- -d $T -C 1 $F -O analog
    emu_options

    # An unplug.
    emu_options length=16
    emu_case "unplug in a frame usb" --oxrok-fails "unplug" --unplug in=2:42 -- -d $U -C 1,2 $F -O analog
    emu_case "unplug in a frame tcp" --oracle-bug "$SIGPIPE" --oxrok-fails "unplug" --unplug tcp-in=42 -- -d $T -C 1,2 $F -O analog
    emu_options

    # The USB shell's faults.
    emu_options empty-at=1
    emu_case "usbtmc empty transfer" --keep-header -- -d $U --scan
    emu_options stall=caps
    emu_case "usbtmc capabilities stalled" --keep-header -- -d $U --scan
    emu_options bad-tag-at=1
    emu_case "usbtmc wrong tag" --no-output -- -d $U --scan
    emu_options in-stall=1
    emu_case "usbtmc bulk in stalled" --expect-fail -- -d $U --show
    emu_options

    SIGROK_CLI=$DLM_STOCK_CLI
}
