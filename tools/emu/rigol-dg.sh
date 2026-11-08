# Rigol DG function generators (emu/src/models/rigol-dg.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=rigol-dg
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=rigol-dg
EMU_FULL=$EMU_DRIVER
EMU_GCOV=rigol-dg
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=rigol-dg
    T=rigol-dg:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    NUMBER="g_ascii_strtod (protocol.c:185-203, :289) takes what it can of a field and 0 of what it can't, and says nothing; by the guide a field is a number or DEF, so oxrok has no reading"
    SHORT="a short answer stops at protocol.c:183-202 with SR_OK and what the channel had, 0 at first; oxrok has no value to give"
    SINC="rigol_dg_string_to_waveform takes the first prefix, protocol.c:30-36, so SINC is SIN, and the show has Sine's range; oxrok takes the longest"
    FIRST_LOST="protocol.c:274-277 ends the capture at the first read that times out, with exit 0 and no sample; oxrok asks again: the guide's gate times go past 10 s"
    LATER_LOST="protocol.c:274-277 ends the capture at the first read that times out, with exit 0, the limit unmet; oxrok warns and asks again"
    TCP_LATE="C's read has no timeout over TCP (scpi_tcp_raw_read_data) and waits; oxrok's read gives up at 1 s, drops the late answer at its next send and asks again: a reading lost, none shifted"
    UNREF="g_variant_unref(data) with data still NULL (protocol.c:93 after a failed read) is a GLib critical, and sigrok-cli's logger exits at one, ahead of what the read's comment means to ignore and of the close; oxrok goes on and says no value"
    NO_LIMIT="protocol.c:287-288 jumps past the limit check on a short answer, so a stream of them never ends a capture; oxrok ends it"
    STRTOF="sigrok-cli reads a value in single precision (session.c:570), so 12345678.9 goes out as 12345679.000000, where the driver keeps a double; oxrok sends what was asked"
    NOT_FINITE="sigrok-cli's strtof takes it, and it goes out as a word (scpi_send_variadic) to a live output, where the driver has no check; oxrok refuses it and sends nothing"
    NO_VALUE="sigrok-cli takes what is no value (session.c's opt_to_gvar, strtof) and sends 0 to a live output; oxrok refuses it and sends nothing"
    ENABLED_LAST="sigrok-cli turns the output on first, as -c comes; oxrok sets it last, by choice, docs/CLI.md"
    SAYS_NOTHING="C says nothing where oxrok warns; pinned for oxrok's words"
    FUNC_NOT_APPL="api.c:717-720 sends SOURn:APPL:x with no values, which by the DG1000Z guide (p.1-5) takes 1 kHz, 5 Vpp, 0 V and 0 degrees on what may be a live output; oxrok sends SOURn:FUNC x, which keeps what is valid for the new waveform (p.2-97, 2-109)"
    ASKS_LIMITS="the generator clamps a value out of its limits with no error (guide p.2-182, 2-185, 2-110, 2-106), which C sends unchecked (api.c:731-771); oxrok asks X? MIN and X? MAX first and refuses what lies outside"
    CLAMPED="the generator would set the nearest limit in its place with no error (guide p.2-97, 2-155, 2-106), and C sends it (api.c:722-771); oxrok refuses it and nothing goes out"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, so the signal kills it mid capture; oxrok stops, turns the counter off and closes"
    # sigrok-cli doesn't close after a refused start (session.c:896-901) and oxrok does:
    # what follows the stop's COUN:STAT OFF goes.
    NOCLOSE='/434f554e3a53544154204f4646/q'
    # A round is a merged count at a time limit: rounds fold, and the rest stands.
    FOLD_T='/^tcp_out 2a4f50433f0a -> 6$/d
/^tcp_in 2 -> 310a$/d
/^tcp_out 434f554e3a4d4541533f0a -> 11$/d
/^tcp_in 80 -> /d'
    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block, and its recv() has no timeout, so a silent instrument holds sigrok-cli for ever; oxrok's read has the timeout"
    SIGPIPE="sr_tcp_write_bytes is a plain send() (tcp.c) and nothing in libsigrok or sigrok-cli ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli (exit 141); oxrok's write returns EPIPE and the capture fails"
    EMPTY_CRASH="an empty SOUR1:APPL? answer leaves params[0] NULL, so protocol.c:168-169 goes to done with SR_OK and no waveform, and api.c:606-607 reads wf_spec->user_name from NULL: SIGSEGV; oxrok says no usable answer"

    # A DG1062Z: scan, show, on both carriers.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show channel 1" -- -d $U -g 1 --show
    emu_case "show channel 2 tcp" -- -d $T -g 2 --show
    emu_case "get limit_samples" -- -d $T --get limit_samples
    emu_case "usbtmc by address" --keep-header -- -d rigol-dg:conn=usbtmc/$EMU_USB --scan
    emu_case "tcp nothing listens" --no-output -- -d rigol-dg:conn=tcp-raw/127.0.0.1/1 --scan

    # What a channel says.
    for key in enabled pattern output_frequency amplitude offset phase; do
        emu_case "get $key" -- -d $T -g 1 --get $key
    done
    emu_case "get enabled usb" -- -d $U -g 2 --get enabled
    emu_case "get phase channel 2" -- -d $T -g 2 --get phase
    emu_case "get duty on a sine" --no-output --expect-fail -- -d $T -g 1 --get output_duty_cycle
    emu_options wave=SQU wave2=PULSE
    emu_case "get duty on a square" -- -d $T -g 1 --get output_duty_cycle
    emu_case "get duty on a pulse" -- -d $T -g 2 --get output_duty_cycle
    emu_case "show square" -- -d $T -g 1 --show
    emu_options wave=NOISE wave2=DC
    emu_case "show noise" -- -d $T -g 1 --show
    emu_case "show dc" -- -d $T -g 2 --show
    emu_case "get frequency of dc" -- -d $T -g 2 --get output_frequency

    # What a setting sends. Each reads the state first.
    emu_options
    emu_case "set enabled on" --no-output -- -d $T -g 1 --config enabled=on --set
    emu_case "set enabled off" --no-output -- -d $U -g 2 --config enabled=off --set
    emu_case "set pattern" --no-output --oracle-differs "$FUNC_NOT_APPL" -- -d $T -g 1 --config pattern=Square --set
    emu_case "set pattern in any case" --no-output --oracle-differs "$FUNC_NOT_APPL" -- -d $T -g 1 --config pattern=pULSE --set
    # A pattern keeps what was set: C's APPL puts 5 Vpp back, as the guide says.
    emu_options amplitude=1
    emu_case "a pattern keeps the amplitude" --oracle-differs "$FUNC_NOT_APPL" -- -d $T -g 1 --config pattern=Square --get amplitude
    emu_options
    emu_case "set pattern unknown" --no-output --expect-fail -- -d $T -g 1 --config pattern=Triangle --set
    emu_case "set frequency" --no-output -- -d $T -g 1 --config output_frequency=2000.5 --set
    emu_case "set amplitude" --no-output --oracle-differs "$ASKS_LIMITS" -- -d $T -g 2 --config amplitude=1.5 --set
    emu_case "set offset" --no-output --oracle-differs "$ASKS_LIMITS" -- -d $T -g 1 --config offset=-0.25 --set
    emu_case "set phase" --no-output -- -d $T -g 1 --config phase=90 --set
    emu_case "set duty on a sine" --no-output --expect-fail -- -d $T -g 1 --config output_duty_cycle=25 --set
    emu_options wave=SQU wave2=PULSE
    emu_case "set duty on a square" --no-output --oracle-differs "$ASKS_LIMITS" -- -d $T -g 1 --config output_duty_cycle=25 --set
    emu_case "set duty on a pulse" --no-output --oracle-differs "$ASKS_LIMITS" -- -d $T -g 2 --config output_duty_cycle=75 --set
    emu_case "set pulse duty out of range" --no-output --expect-fail --oracle-differs "$CLAMPED" -- -d $T -g 2 --config output_duty_cycle=100 --set
    emu_options wave=NOISE wave2=DC
    emu_case "set frequency on noise" --no-output --expect-fail -- -d $T -g 1 --config output_frequency=1000 --set
    emu_case "set phase on noise" --no-output --expect-fail -- -d $T -g 1 --config phase=10 --set
    emu_case "set amplitude on dc" --no-output --expect-fail -- -d $T -g 2 --config amplitude=1 --set
    emu_case "set offset on dc" --no-output --oracle-differs "$ASKS_LIMITS" -- -d $T -g 2 --config offset=1 --set
    emu_options
    # Refused by the unit, in its queue: neither tool reads it.
    emu_case "set frequency out of range" --no-output --expect-fail --oracle-differs "$CLAMPED" -- -d $T -g 1 --config output_frequency=100000000 --set
    emu_case "set frequency out of range says why" --expect-fail --oracle-differs "$SAYS_NOTHING" --stderr -- -d $T -g 1 --config output_frequency=100000000 --set
    emu_case "set phase out of range" --no-output --expect-fail --oracle-differs "$CLAMPED" -- -d $T -g 1 --config phase=400 --set
    emu_case "set amplitude above the instrument's" --no-output --expect-fail --oracle-differs "$ASKS_LIMITS" -- -d $T -g 1 --config amplitude=50 --set
    emu_case "set offset above the instrument's" --no-output --expect-fail --oracle-differs "$ASKS_LIMITS" -- -d $T -g 1 --config offset=4 --set
    emu_options limits=silent
    emu_case "set amplitude with no limits to be had" --oracle-differs "$SAYS_NOTHING" --stderr -- -d $U -g 1 --config amplitude=1 --set
    emu_options
    emu_case "set in double precision" --no-output --oracle-differs "$STRTOF" -- -d $T -g 1 --config output_frequency=12345678.9 --set
    emu_case "set a NaN" --no-output --expect-fail --oracle-differs "$NOT_FINITE" -- -d $T -g 1 --config amplitude=nan --set
    emu_case "set an infinity" --no-output --expect-fail --oracle-differs "$NOT_FINITE" -- -d $T -g 1 --config offset=inf --set
    emu_case "set no number" --no-output --expect-fail --oracle-differs "$NO_VALUE" -- -d $T -g 1 --config amplitude=abc --set
    emu_case "set enabled goes last" --no-output --oracle-differs "$ENABLED_LAST and $ASKS_LIMITS" -- -d $T -g 1 --config enabled=on --config amplitude=1 --set
    emu_case "set several in order" --no-output --oracle-differs "$ASKS_LIMITS" -- -d $T -g 1 --config output_frequency=2000 --config amplitude=1 --config enabled=on --set

    # A capture.
    emu_case "two frames" --record -- -d $U --samples 2 -O analog
    emu_case "two frames tcp" -- -d $T --samples 2 -O analog
    emu_case "a frame with the header" --keep-header -- -d $T --samples 1 -O analog
    emu_case "only the counter's frequency" -- -d $T -C FREQ1 --samples 1 -O analog
    emu_case "only a generator's channel" -- -d $T -C CH1 --samples 1 -O analog
    emu_case "time shorter than the poll" -- -d $T --time 20 -O analog
    emu_case "time of some rounds" --filter '5,$d' --tx-filter "$FOLD_T" -- -d $T --time 350 -O analog
    emu_options mark-at=12
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --timeout 15 --interrupt tcp-model --filter '5,$d' --tx-filter "/^interrupt\$/,\$ {
$FOLD_T
}" -- -d $T --continuous -O analog
    emu_options
    emu_case "tcp-rigol nothing listens" --no-output -- -d rigol-dg:conn=tcp-rigol/127.0.0.1/1 --scan
    emu_options framing=rigol
    emu_case "tcp-rigol capture" -- -d rigol-dg:conn=tcp-rigol/127.0.0.1/$EMU_TCP_PORT --samples 1 -O analog
    emu_options
    emu_options counter=RUN
    emu_case "counter already running" -- -d $T --samples 2 -O analog
    emu_options counter=STOP
    emu_case "counter stopped" -- -d $T --samples 1 -O analog
    emu_options model=DG1022Z
    emu_case "dg1022z capture" -- -d $T --samples 1 -O analog
    # Their counter takes a second each way, and channel 2 stays as it is: api.c:902-903
    # makes 2 & 1 of the quirk, so that its check never holds.
    emu_options model=DG811
    emu_case "dg811 capture" -- -d $T --samples 1 -O analog
    emu_options model=DG812 output2=1
    emu_case "dg812 capture with channel 2 on" -- -d $T --samples 1 -O analog
    emu_case "dg812 says what it leaves" --oracle-differs "$SAYS_NOTHING" --stderr -- -d $T --samples 1 -O analog
    emu_case "dg812 get enabled" -- -d $T -g 2 --get enabled
    emu_options model=DG992 counter=RUN
    emu_case "dg992 counter running" -- -d $T --samples 1 -O analog

    # Models.
    for m in DG1062Z DG1032Z DG1022Z; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "frequency range $m" -- -d $T -g 1 --show
    done
    for m in DG811 DG812 DG821 DG822 DG831 DG832; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "frequency range $m" -- -d $T -g 1 --show
    done
    for m in DG952 DG972 DG992; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "frequency range $m" -- -d $T -g 1 --show
    done
    for m in MSO5354 MSO5204 MSO5104 MSO5102 MSO5074 MSO5072; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
    done

    # An MSO5000's generator: another table, no counter.
    emu_options model=MSO5074
    emu_case "mso show" --keep-header -- -d $T --show
    emu_case "mso show channel 1" -- -d $T -g 1 --show
    emu_case "mso get enabled" -- -d $T -g 1 --get enabled
    emu_case "mso get frequency" -- -d $T -g 2 --get output_frequency
    emu_case "mso set enabled" --no-output -- -d $T -g 1 --config enabled=on --set
    emu_case "mso set pattern" --no-output -- -d $T -g 1 --config pattern=ExpRise --set
    emu_case "mso set frequency" --no-output -- -d $T -g 1 --config output_frequency=500 --set
    emu_case "mso capture with no key for it" --no-output --expect-fail -- -d $T --samples 1 -O analog
    emu_options model=MSO5074 wave=PULS
    emu_case "mso get duty on a pulse" -- -d $T -g 1 --get output_duty_cycle
    emu_case "mso set amplitude says it clamps" --oracle-differs "$SAYS_NOTHING" --stderr -- -d $T -g 1 --config amplitude=1 --set
    emu_case "mso set duty on a pulse" --no-output -- -d $T -g 1 --config output_duty_cycle=30 --set
    emu_options model=MSO5074 wave=SQU
    emu_case "mso get duty on a square" --no-output --expect-fail -- -d $T -g 1 --get output_duty_cycle
    emu_case "mso set duty on a square" --no-output --expect-fail -- -d $T -g 1 --config output_duty_cycle=30 --set
    emu_options model=MSO5074 wave=SINC
    emu_case "mso sinc" --oracle-differs "$SINC" -- -d $T -g 1 --get pattern
    emu_case "mso frequency range of sinc" --oracle-differs "$SINC" -- -d $T -g 1 --show

    # What the instrument says at a scan.
    emu_options 'say=*IDN?=RIGOL~TECHNOLOGIES,dg1062z,X,1'
    emu_case "scan in another case" --keep-header -- -d $T --scan
    emu_options 'say=*IDN?=Rigol~Technologies,DG1062Z,1'
    emu_case "scan three fields" --keep-header -- -d $T --scan
    emu_options 'say=*IDN?=Rigol~Technologies,DG1062'
    emu_case "scan two fields" --no-output -- -d $T --scan
    emu_options idn=Acme,X,1,2
    emu_case "scan another instrument" --no-output -- -d $U --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options silent-at=1
    emu_case "scan silent once" --no-output -- -d $U --scan
    emu_options opc-late=3
    emu_case "opc says 0 three times" --keep-header -- -d $U --scan
    emu_options remote=1
    emu_case "remote control" --keep-header -- -d $U --scan

    # A channel's state, as the instrument may say it.
    emu_options 'say=SOUR1:APPL?=SIN,1.000000E+03,5.000000E+00,0.000000E+00,0.000000E+00'
    emu_case "apply with no quotes" -- -d $T -g 1 --get output_frequency
    emu_options 'say=SOUR1:APPL?="SINE,1.5E+03,2,3,4,5"'
    emu_case "apply long" -- -d $T -g 1 --get phase
    emu_options 'say=SOUR1:APPL?="FOO,1,2,3,4"'
    emu_case "apply unknown waveform" --no-output --expect-fail -- -d $T -g 1 --get output_frequency
    emu_case "set on an unknown waveform" --no-output --expect-fail -- -d $T -g 1 --config output_frequency=1 --set
    emu_options 'say=SOUR1:APPL?="SIN,1000"'
    emu_case "apply short" --no-output --expect-fail --oracle-differs "$SHORT" -- -d $T -g 1 --get amplitude
    emu_case "apply short the field it has" -- -d $T -g 1 --get output_frequency
    emu_options 'say=SOUR1:APPL?="SIN,abc,2,3,4"'
    emu_case "apply no number" --no-output --expect-fail --oracle-differs "$NUMBER" -- -d $T -g 1 --get output_frequency
    emu_options 'say=SOUR1:APPL?="SIN,1.5xyz,2,3,4"'
    emu_case "apply half a number" --no-output --expect-fail --oracle-differs "$NUMBER" -- -d $T -g 1 --get output_frequency
    emu_options 'say=OUTP1:STATE?=maybe'
    emu_case "enabled says neither" --no-output --expect-fail -- -d $T -g 1 --get enabled
    emu_options 'say=SOUR1:FUNC:SQU:DCYC?=abc' wave=SQU
    emu_case "duty no number" --no-output --expect-fail -- -d $T -g 1 --get output_duty_cycle
    emu_options 'say=SOUR1:APPL?='
    emu_case "apply empty get pattern" --no-output --expect-fail --oracle-bug "$EMPTY_CRASH" -- -d $T -g 1 --get pattern
    emu_options silent-at=4
    emu_case "apply silent" --no-output --expect-fail -- -d $U -g 1 --get output_frequency
    emu_case "apply silent tcp" --no-output --expect-fail --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 5 -- -d $T -g 1 --get output_frequency
    emu_case "set with the state silent" --no-output --expect-fail -- -d $U -g 1 --config amplitude=1 --set
    emu_options model=MSO5074 'say=SOUR1:FREQ?=abc'
    emu_case "mso frequency no number" --no-output --expect-fail --oracle-differs "$NUMBER" -- -d $T -g 1 --get output_frequency
    emu_options model=MSO5074 'say=SOUR1:FUNC?=FOO'
    emu_case "mso unknown waveform" --no-output --expect-fail -- -d $T -g 1 --get pattern
    emu_options model=MSO5074 silent-at=5
    emu_case "mso frequency silent" --no-output --expect-fail --oracle-bug "$UNREF" -- -d $U -g 1 --get output_frequency

    # A capture's faults. Queries: 1 *IDN?, 2 *OPC?, 3 COUN:STAT?, 4 *OPC?, 5 *OPC?,
    # 6 COUN:MEAS?, 7 *OPC?, 8 COUN:MEAS?...
    emu_options 'say=COUN:STAT?=NOPE'
    emu_case "counter says something else" -- -d $T --samples 1 -O analog
    emu_options 'play=1,2,3'
    emu_case "measure short" -- -d $T --samples 1 -O analog
    emu_options 'play=;'
    emu_case "measure empty" -- -d $T --samples 1 -O analog
    emu_options 'play=1,2,3,4,5,6'
    emu_case "measure with six" -- -d $T --samples 1 -O analog
    emu_options 'play=a,b,c,d,e'
    emu_case "measure no number" --oracle-differs "$NUMBER" -- -d $T --samples 1 -O analog
    emu_options silent-at=3
    emu_case "counter state silent" --no-output --expect-fail --oxrok-fails "No answer about the counter's state" --tx-filter "$NOCLOSE" -- -d $U --samples 1 -O analog
    emu_options silent-at=6
    emu_case "first measure silent" --oracle-differs "$FIRST_LOST" -- -d $U --samples 1 -O analog
    emu_case "first measure silent tcp" --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 5 -- -d $T --samples 1 -O analog
    emu_options silent-at=7
    emu_case "an opc silent in a capture" -- -d $U --samples 2 -O analog
    emu_options silent-at=8
    emu_case "second measure silent says so" --oracle-differs "$SAYS_NOTHING" --stderr -- -d $U --samples 2 -O analog
    emu_case "second measure silent" --oracle-differs "$LATER_LOST" -- -d $U --samples 2 -O analog
    emu_options late-at=8:80
    emu_case "second measure late tcp" --oracle-differs "$TCP_LATE" -- -d $T --samples 3 -O analog
    emu_case "second measure late" --oracle-differs "$LATER_LOST" -- -d $U --samples 3 -O analog
    emu_options late-at=7:80
    emu_case "an opc late in a capture" -- -d $U --samples 2 -O analog
    emu_case "an opc late in a capture tcp" -- -d $T --samples 2 -O analog
    emu_options opc-late=3
    emu_case "capture opc says 0" -- -d $T --samples 1 -O analog
    # Over 10 s: the one case that a stream of what makes no reading ends a capture.
    emu_options 'say=COUN:MEAS?=1,2,3'
    FOLD_S='/^tcp_out 2a4f50433f0a -> 6$/d
/^tcp_in 2 -> 310a$/d
/^tcp_out 434f554e3a4d4541533f0a -> 11$/d
/^tcp_in 6 -> 312c322c330a$/d'
    emu_case "measure short for ever" --no-output --oracle-bug "$NO_LIMIT" --oracle-hangs --timeout 15 --oxrok-fails "no usable answer from the counter in 10 s" --tx-filter "$FOLD_S" -- -d $T --samples 1 -O analog
    emu_case "measure short under a time limit" --no-output --oracle-bug "$NO_LIMIT" --oracle-hangs --timeout 4 --tx-filter "$FOLD_S" -- -d $T --time 300 -O analog

    # Gone.
    emu_options
    emu_case "unplug in a capture" --oxrok-fails "unplug" --unplug in=2:12 -- -d $U --samples 100 -O analog
    emu_case "unplug in a capture tcp" --oracle-bug "$SIGPIPE" --oxrok-fails "unplug" --unplug tcp-in=8 -- -d $T --samples 100 -O analog
}
