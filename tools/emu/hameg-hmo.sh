# Hameg and Rohde & Schwarz scopes (emu/src/models/hameg-hmo.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=hameg-hmo
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=hameg-hmo
EMU_FULL=$EMU_DRIVER
EMU_GCOV=hameg-hmo
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    # A capture's output in few lines: every 64th, and through --digest the hash of all.
    # What C can't show. Its analog packets are empty (see ANALOG_EMPTY), so a case about
    # the wire, or about logic, takes oxrok's analog lines out, and they have their own
    # cases, with AO.
    AO='1~64!d'
    A='/^CH[1-4]: /d
/^(libsigrok|oxrok) [0-9]/d'
    # By scan: a re-attach moves the address, so conn=usbtmc/$EMU_USB only holds with no
    # options changed.
    U=hameg-hmo
    T=hameg-hmo:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    # C lists a class among its device options (protocol.c:118), and sigrok-cli prints
    # that as "(null): ". oxrok has no such key: this one-sided rule takes away C's line,
    # and "show has a null in C" pins it.
    NULLKEY='/^    \(null\): $/d'
    NULLTEXT="devopts starts with SR_CONF_OSCILLOSCOPE (protocol.c:118), a class and no option, so sigrok-cli's --show prints '(null): ' for it; oxrok doesn't"
    ANALOG_EMPTY="protocol.c:1521-1527 sets analog.data and num_samples and then calls sr_analog_init, whose memset clears them (analog.c), so since 8c381a3 every analog packet is empty and sigrok-cli prints no sample; oxrok sends what the scope sent. So analog wire cases here use -O bits, which print no analog sample on either side"
    FRAMES_ALONE="protocol.c:1616 stops at num_samples >= samples_limit, true with no sample limit, so --frames N alone gives one frame; d779dca turned == frame_limit into this when it added the sample limit. oxrok gives N"
    COMPACT="protocol.c:1197 looks the timebase up in the long table on every model and uses the index in the model's own (api.c:175), which a058de0 made one shorter on a compact HMO, so 1 ms reads as 2 ms. oxrok reads it as the scope said"
    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block (nonblocking FALSE, scpi_tcp.c), and its recv() has no timeout, so sr_scpi_get_string's 1 s (scpi.c) never runs and a silent instrument holds sigrok-cli for ever; oxrok's read has the timeout"
    SIGPIPE="sr_tcp_write_bytes is a plain send() (tcp.c) and nothing in libsigrok or sigrok-cli ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli (exit 141); oxrok's write returns EPIPE and the capture fails"
    TCP_EOF="sr_tcp_read_bytes returns the 0 of a closed connection as a read of nothing (tcp.c), and neither sr_scpi_get_block nor hmo_receive_data takes it for an end, so the capture goes on reading for ever; oxrok says the instrument closed the connection"
    NEVER_ENDS="hmo_receive_data returns TRUE on a failed sr_scpi_get_block (protocol.c:1513) and its source stays, with no bound, so a device that gives no block holds the capture for ever; oxrok's capture fails"
    CUT_USB="a USBTMC message that ends ahead of the block's length: the next read is past EOM, an error, so sr_scpi_get_block drops all it has (scpi.c) and hmo_receive_data asks again for ever; oxrok hands on what came, as C does on a timeout, and goes on"
    CUT_TCP="a block that ends ahead of its length over tcp-raw: recv() has no timeout (scpi_tcp.c), so sigrok-cli waits for the rest for ever; oxrok hands on what came and goes on"
    ZERO="sr_scpi_get_block returns SR_OK with no array at a length of 0 (scpi.c), and protocol.c:1521 reads data->data through it: sigrok-cli dies; oxrok goes on with an empty block"
    PATTERN_CUT="api.c:385 formats the pattern command into MAX_COMMAND_SIZE, 128 bytes (protocol.h:34): 107 bytes of it come before the pattern, so a pattern of 20 leaves no room for its closing quote, and C sends ':TRIG:A:PATT:SOUR \"10101010101010101010' with exit 0; oxrok sends the quote"
    # A block that stops partway, as a scope may: C over tcp-raw waits for the rest, and
    # so does oxrok, up to its bound with no byte.
    RTH_CRASH="on an RTH, with USER2 in its table and the second dialect, api.c:500-502 gives ':DIG%d:THR %s' the arguments (idx, 2, float_str), so %s reads 2 as a pointer: sigrok-cli dies; oxrok sends the level"

    # While nothing comes on USBTMC, a request goes for each read's second, so how many
    # is the clock's. This keeps the first of a run of them.
    FOLD='/ffffff7f00000000 -> 12$/{
x
/R/{
x
d
}
s/.*/R/
x
b
}
x
s/.*//
x'

    # The scope, on both carriers.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header --filter "$NULLKEY" -- -d $U --show
    emu_case "show tcp" --keep-header --filter "$NULLKEY" -- -d $T --show
    emu_case "show has a null in C" --keep-header --oracle-differs "$NULLTEXT" -- -d $T --show
    emu_case "show CH1" -- -d $T -g CH1 --show
    emu_case "show POD1" -- -d $T -g POD1 --show
    emu_case "show POD2" -- -d $T -g POD2 --show

    # Each value.
    for GET_KEY in timebase samplerate num_hdiv horiz_triggerpos triggersource triggerslope triggerpattern highresolution peakdetection limit_frames; do
        emu_case "get $GET_KEY" -- -d $T --get $GET_KEY
    done
    emu_case "get limit_samples is no get" --no-output --expect-fail -- -d $T --get limit_samples
    emu_case "get vdiv" -- -d $T -g CH2 --get vdiv
    emu_case "get num_vdiv" -- -d $T -g CH2 --get num_vdiv
    emu_case "get coupling" -- -d $T -g CH2 --get coupling
    emu_case "get vdiv needs a group" --no-output --expect-fail -- -d $T --get vdiv
    emu_case "get logic_threshold" -- -d $T -g POD2 --get logic_threshold
    emu_case "get custom threshold not in custom mode" --no-output --expect-fail -- -d $T -g POD1 --get logic_threshold_custom
    emu_case "no such group" --expect-fail -- -d $T -g POD3 --show

    # Each setting.
    emu_case "set timebase" --no-output -- -d $U --config timebase=2ms --set
    emu_case "set timebase tcp" --no-output -- -d $T --config timebase=500us --set
    emu_case "set horiz_triggerpos" --no-output -- -d $T --config horiz_triggerpos=0.25 --set
    emu_case "set horiz_triggerpos centre" --no-output -- -d $T --config horiz_triggerpos=0.5 --set
    emu_case "set horiz_triggerpos out of range" --no-output --expect-fail -- -d $T --config horiz_triggerpos=2 --set
    emu_case "set trigger source" --no-output -- -d $T --config triggersource=D3 --set
    emu_case "set trigger slope" --no-output -- -d $T --config triggerslope=NEG --set
    emu_case "set trigger pattern" --no-output -- -d $T --config triggerpattern=10X --set
    emu_case "set trigger pattern too long" --no-output --expect-fail -- -d $T --config triggerpattern=101010101010101010101 --set
    # C's 128 bytes end before the closing quote: 107, 20, and the quote. oxrok's .tx keeps
    # the quote; C's line is only in a run (its .oracle is its empty stdout).
    emu_case "set trigger pattern at its longest" --no-output --oracle-differs "$PATTERN_CUT" -- -d $T --config triggerpattern=10101010101010101010 --set
    emu_case "set high resolution on" --no-output -- -d $T --config highresolution=on --set
    emu_case "set high resolution off" --no-output -- -d $T --config highresolution=off --set
    emu_case "set peak detection on" --no-output -- -d $T --config peakdetection=on --set
    emu_case "set peak detection off" --no-output -- -d $T --config peakdetection=off --set
    emu_case "set vdiv" --no-output -- -d $T -g CH2 --config vdiv=500mV --set
    emu_case "set vdiv needs a group" --no-output --expect-fail -- -d $T --config vdiv=500mV --set
    emu_case "set vdiv not on a pod" --no-output --expect-fail -- -d $T -g POD1 --config vdiv=500mV --set
    emu_case "set vdiv not in the table" --no-output --expect-fail -- -d $T -g CH1 --config vdiv=20V --set
    emu_case "set coupling" --no-output -- -d $T -g CH1 --config coupling=AC --set
    emu_case "set logic_threshold" --no-output -- -d $T -g POD2 --config logic_threshold=CMOS --set
    emu_case "set custom threshold" --no-output -- -d $T -g POD2 --config logic_threshold_custom=1.5 --set
    emu_case "set custom threshold out of range" --no-output --expect-fail -- -d $T -g POD1 --config logic_threshold_custom=9 --set

    # Captures. Two channels are on at power on, as a scope says at the open.
    emu_case "frame" --oracle-differs "$ANALOG_EMPTY" --keep-header --digest --filter "$AO" -- -d $U --frames 1 -O analog
    emu_case "frame tcp" --oracle-differs "$ANALOG_EMPTY" --keep-header --digest --filter "$AO" -- -d $T --frames 1 -O analog
    emu_case "frame wire" --keep-header --filter "$A" -- -d $U --frames 1 -O bits
    emu_case "frame wire tcp" --keep-header --filter "$A" -- -d $T --frames 1 -O bits
    emu_case "one channel off" --no-output --filter "$A" -- -d $T -C CH2 --frames 1 -O bits
    emu_case "all analog" --no-output --filter "$A" -- -d $T -C CH1,CH2,CH3,CH4 --frames 1 -O bits
    emu_case "frames alone" --filter "$A" --oracle-differs "$FRAMES_ALONE" -- -d $T -C D0 --frames 3 -O bits
    emu_case "frames and a high limit" --no-output --filter "$A" -- -d $T -C CH1 --frames 2 --samples 5000 -O bits
    emu_case "frames and a low limit" --oracle-differs "$ANALOG_EMPTY" --digest --filter "$AO" -- -d $T -C CH1 --frames 2 --samples 10 -O analog
    emu_case "samples" --oracle-differs "$ANALOG_EMPTY" -- -d $T -C CH1 --samples 10 -O analog
    emu_case "samples above a frame" --no-output --filter "$A" -- -d $T -C CH1 --samples 5000 -O bits
    emu_case "time" --no-output --filter "$A" -- -d $T -C CH1 --time 1ms -O bits
    emu_case "continuous" --no-output --expect-fail -- -d $T -C CH1 --continuous -O bits
    emu_case "pod one" --record --filter "$A" -- -d $U -C D0 --frames 1 -O bits
    emu_case "pod one tcp" --filter "$A" -- -d $T -C D0,D7 --frames 1 -O bits
    emu_case "pod two alone" --filter "$A" -- -d $T -C D8 --frames 1 -O bits
    emu_case "both pods" --keep-header --filter "$A" -- -d $T -C D0,D15 --frames 1 -O bits
    emu_case "both pods samples" -- -d $T -C D0,D8 --samples 8 -O bits
    emu_case "pod samples" -- -d $T -C D0 --samples 8 -O bits
    emu_case "channel and pod" --keep-header --filter "$A" -- -d $T -C CH1,D0 --frames 1 -O bits
    emu_case "channel and pod, analog" --filter "$A" -- -d $T -C CH1,D0 --frames 1 -O bits
    emu_case "CH3 with pod one" --no-output --expect-fail -- -d $T -C CH3,D0 --frames 1 -O bits
    emu_case "CH4 with pod two" --no-output --expect-fail -- -d $T -C CH4,D8 --frames 1 -O bits

    # What the scope says at the open.
    emu_options set=LOG0:STAT=1 set=POD1:STAT=1
    emu_case "on at the open" --filter "$A" -- -d $T --frames 1 -O bits
    emu_options set=PROB1:SET:ATT:UNIT=A
    emu_case "a current probe" --oracle-differs "$ANALOG_EMPTY" --digest --filter "$AO" -- -d $T -C CH1 --frames 1 -O analog
    emu_options set=POD1:THR=USER2
    emu_case "get custom threshold" -- -d $T -g POD1 --get logic_threshold_custom
    emu_options set=POD1:THR=USER1
    emu_case "threshold USER1" -- -d $T -g POD1 --show
    emu_options set=POD1:THR=USER
    emu_case "threshold USER is USER1" -- -d $T -g POD1 --show
    emu_options set=POD1:THR=BOGUS
    emu_case "threshold nobody knows" --expect-fail --filter "$NULLKEY" -- -d $T --show
    emu_options set=POD1:THR:UDL2=x set=POD1:THR=USER2
    emu_case "a level that is no number" --expect-fail --filter "$NULLKEY" -- -d $T --show
    emu_options set=TRIG:A:SOUR=PATT
    emu_case "the trigger is a pattern" -- -d $T --get triggersource
    emu_options set=ACQ:HRES=AUTO
    emu_case "high resolution is anything but OFF" -- -d $T --get highresolution
    emu_options set=ACQ:PEAK=ON
    emu_case "peak detection is anything but OFF" -- -d $T --get peakdetection
    emu_options set=TIM:DIV=10
    emu_case "ten divisions" -- -d $T --get num_hdiv
    emu_case "ten divisions set position" --no-output -- -d $T --config horiz_triggerpos=0.75 --set
    emu_options set=TIM:POS=3.000E-03
    emu_case "a trigger position" -- -d $T --get horiz_triggerpos
    emu_options set=TRIG:A:PATT:SOUR=10X
    emu_case "a pattern with no quotes" -- -d $T --get triggerpattern
    emu_options 'set=TRIG:A:PATT:SOUR="1010101010101010101010101"'
    emu_case "a long pattern" -- -d $T --get triggerpattern
    emu_options eol=crlf
    emu_case "crlf" --keep-header --filter "$NULLKEY" -- -d $T --show
    for BAD in CHAN1:COUP=BOGUS CHAN1:SCAL=3.000E+00 CHAN1:SCAL=volts CHAN1:POS=x CHAN1:STAT=maybe TIM:SCAL=3.000E-03 TIM:DIV=twelve TIM:POS=x TRIG:A:SOUR=NOPE TRIG:A:EDGE:SLOP=UP LOG0:STAT=maybe POD1:STAT=maybe; do
        emu_options set=$BAD
        emu_case "refused at the open $BAD" --expect-fail --filter "$NULLKEY" -- -d $T --show
    done

    # Blocks in other forms.
    emu_options block-lf=0
    emu_case "no line end after a block" --no-output --filter "$A" -- -d $U --frames 1 -O bits
    emu_case "no line end after a block tcp" --no-output --filter "$A" -- -d $T --frames 1 -O bits
    emu_options digits=9
    emu_case "nine digits" --no-output --filter "$A" -- -d $T -C CH1 --frames 1 -O bits
    emu_options odd-at=1
    emu_case "a byte over" --no-output --filter "$A" -- -d $T -C CH1 --frames 1 -O bits
    emu_options points2=500
    emu_case "pod two shorter" --filter "$A" -- -d $T -C D0,D8 --frames 1 -O bits
    emu_options points2=1010
    emu_case "pod two longer" --filter "$A" -- -d $T -C D0,D8 --frames 1 -O bits

    # How a USBTMC device is found and held.
    emu_reset
    emu_case "usbtmc by address" --keep-header -- -d hameg-hmo:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header -- -d hameg-hmo:conn=usbtmc --scan
    emu_case "usbtmc kernel driver bound" --no-output --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' --filter "$A" -- -d $U -C CH1 --frames 1 -O bits
    emu_case "tcp nothing listens" --no-output -- -d hameg-hmo:conn=tcp-raw/127.0.0.1/1 --scan
    emu_options remote=1
    emu_case "remote control" --keep-header -- -d $U --scan
    emu_case "remote control in a frame" --no-output --filter "$A" -- -d $U -C CH1 --frames 1 -O bits

    # Who answers.
    emu_options idn=Acme,HMO3054,1,2
    emu_case "scan another maker" --no-output -- -d $T --scan
    emu_options idn=HAMEG,HMO9999,1,2
    emu_case "scan another model" --no-output -- -d $T --scan
    emu_options idn=HAMEG,hmo3054,1,2
    emu_case "scan a name in lower case" --no-output -- -d $T --scan
    emu_options idn=HAMEG,HMO3054,1.0
    emu_case "scan with no serial" --keep-header -- -d $T --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $U --scan

    # Silence and lateness at the open and in a setting.
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options silent-at=1
    emu_case "scan silent once" --no-output -- -d $U --scan
    emu_options mute-at=3
    emu_case "show goes silent" --expect-fail --filter "$NULLKEY" -- -d $U --show
    emu_options late-at=3:5
    emu_case "show late" --keep-header --filter "$NULLKEY" -- -d $U --show
    emu_case "show late tcp" --keep-header --filter "$NULLKEY" -- -d $T --show
    emu_options opc-late=20
    emu_case "opc says 0 a while" --no-output -- -d $T --config triggerslope=NEG --set

    # Blocks that don't come, or not whole. The scan and the open take 50 queries here, so
    # the first request is the 51st.
    emu_options late-at=51:5
    emu_case "block late" --no-output --filter "$A" -- -d $T --frames 1 -O bits
    emu_case "block late usb" --no-output --filter "$A" -- -d $U --frames 1 -O bits
    # 150 ticks of 20 ms on TCP.
    emu_options pause-at=1:150
    emu_case "a block that pauses 3 s tcp" --filter "$A" -- -d $T -C D0 --frames 1 -O bits
    emu_options odd-at=2
    emu_case "a byte over in the second channel" --no-output --filter "$A" -- -d $T --frames 1 -O bits

    # C can't come back from these.
    emu_options mute-at=1
    emu_case "scan silent tcp" --no-output --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --scan
    emu_options mute-at=3
    emu_case "show goes silent tcp" --expect-fail --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --show
    # 30 s, the only run of the bound's whole wiring and words through the tool.
    # -C CH1 turns CH2 off, so the 51st query is the rate, and the block the 52nd.
    emu_options mute-at=52
    emu_case "no block usb" --no-output --tx-filter "$FOLD" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 40 --oxrok-fails "has sent no block" -- -d $U -C CH1 --frames 1 -O bits
    emu_options mute-at=51
    emu_case "no rate at the start" --no-output --expect-fail -- -d $U -C CH1 --frames 1 -O bits
    emu_options cut-at=1
    emu_case "block cut usb" --filter "$A" --oracle-bug "$CUT_USB" --oracle-hangs --timeout 6 -- -d $U -C D0 --frames 1 -O bits
    # oxrok waits the 30 s of its bound with no byte inside a block, then hands on what came.
    emu_case "block cut tcp" --filter "$A" --oracle-bug "$CUT_TCP" --oracle-hangs --timeout 45 -- -d $T -C D0 --frames 1 -O bits
    emu_options zero-at=1
    emu_case "empty block" --filter "$A" --oracle-bug "$ZERO" -- -d $T -C CH1,D0 --frames 1 -O bits
    emu_options junk-at=1
    emu_case "text for a block" --no-output --tx-filter "$FOLD" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 8 --oxrok-fails "No usable answer" -- -d $U -C CH1 --frames 1 -O bits
    emu_options indefinite-at=1
    emu_case "indefinite block" --no-output --tx-filter "$FOLD" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 8 --oxrok-fails "No usable answer" -- -d $U -C CH1 --frames 1 -O bits
    emu_options bad-len-at=1
    emu_case "a length that is no number" --no-output --tx-filter "$FOLD" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 8 --oxrok-fails "No usable answer" -- -d $U -C CH1 --frames 1 -O bits

    # The other series.
    emu_options model=HMO722
    emu_case "HMO722 show" --keep-header --filter "$NULLKEY" --oracle-differs "$COMPACT" -- -d $T --show
    emu_case "HMO722 POD1" -- -d $T -g POD1 --show
    for SCOPE in HMO1002 RTC1002 HMO3032 RTB2002 RTB2004 RTM3002 RTM3004 RTA4004 RTH1002 RTH1004; do
        emu_options model=$SCOPE
        emu_case "$SCOPE show" --keep-header --filter "$NULLKEY" -- -d $T --show
        emu_case "$SCOPE POD1" -- -d $T -g POD1 --show
    done
    emu_options model=HMO1002
    emu_case "hmo1002 frame" --oracle-differs "$ANALOG_EMPTY" --digest --filter "$AO" -- -d $T -C CH2,D0 --frames 1 -O analog
    emu_case "hmo1002 set custom" --no-output -- -d $T -g POD1 --config logic_threshold_custom=2.5 --set
    emu_case "hmo1002 one pod" --expect-fail -- -d $T -g POD2 --show
    emu_options model=RTB2004
    emu_case "rtb2004 CH1 and both pods" --filter "$A" -- -d $T -C CH1,D0,D15 --frames 1 -O bits
    emu_case "rtb2004 POD2" -- -d $T -g POD2 --show
    emu_case "rtb2004 set threshold" --no-output -- -d $T -g POD2 --config logic_threshold=ECL --set
    emu_case "rtb2004 set custom" --no-output -- -d $T -g POD2 --config logic_threshold_custom=1.5 --set
    emu_case "rtb2004 set 50 V" --no-output -- -d $T -g CH1 --config vdiv=50V --set
    emu_case "rtb2004 set coupling refused" --no-output --expect-fail -- -d $T -g CH1 --config coupling=AC --set
    emu_options model=RTB2004 set=DIG8:TECH=MAN
    emu_case "rtb2004 custom threshold" -- -d $T -g POD2 --get logic_threshold_custom
    emu_options model=RTH1002
    emu_case "rth1002 frame" --filter "$A" -- -d $T -C CH1,D3 --frames 1 -O bits
    emu_case "rth1002 set threshold" --no-output -- -d $T -g POD1 --config logic_threshold=USER2 --set
    emu_options model=RTH1002 set=DIG1:TECH=USER2
    emu_case "rth1002 show custom" -- -d $T -g POD1 --show
    emu_options model=HMO1024
    emu_case "compact show" --keep-header --filter "$NULLKEY" --oracle-differs "$COMPACT" -- -d $T --show
    emu_case "compact get timebase" --oracle-differs "$COMPACT" -- -d $T --get timebase
    emu_case "compact set timebase" --no-output -- -d $T --config timebase=2ms --set
    emu_case "compact frame" --no-output --filter "$A" -- -d $T -C CH1 --frames 1 -O bits
    emu_case "compact has no 1 ns" --no-output --expect-fail -- -d $T --config timebase=1ns --set
    emu_options model=RTH1002
    emu_case "rth1002 set custom" --no-output --oracle-bug "$RTH_CRASH" -- -d $T -g POD1 --config logic_threshold_custom=2.5 --set

    # An unplug. On TCP the instrument closes the connection.
    emu_options
    # After the readback, so the writes of the capture's start come next: C dies. A
    # setting's write is the first there is with no channel to change.
    emu_case "unplug before a capture tcp" --no-output --oracle-bug "$SIGPIPE" --unplug tcp-in=50 --oxrok-fails "Broken pipe" -- -d $T -C CH1,CH3 --frames 2 -O bits
    # After a block: C reads 0 bytes for ever, which is no error to it.
    emu_case "unplug in a frame tcp" --no-output --filter "$A" --oracle-bug "$TCP_EOF" --oracle-hangs --timeout 4 --unplug tcp-in=51 --oxrok-fails "closed|Broken pipe" -- -d $T --frames 1 -O bits
    emu_case "unplug in a frame" --no-output --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 6 --unplug in=2:52 --oxrok-fails "unplug" --filter "$A" -- -d $U --frames 2 --samples 5000 -O bits
    # A stop in a wait for a block: it ends at once, with no failure.
    emu_options mute-at=52
    emu_case "interrupt in a silent wait" --no-output --filter "$A" --oracle-bug "sigrok-cli has no SIGINT handler, so it dies at the signal; oxrok ends its capture and closes" --interrupt tcp-in=51 -- -d $T --frames 1 -O bits
    emu_options
}
