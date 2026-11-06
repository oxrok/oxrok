# LeCroy X-Stream scopes (emu/src/models/lecroy-xstream.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=lecroy-xstream
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=lecroy-xstream
EMU_FULL=$EMU_DRIVER
EMU_GCOV=lecroy-xstream
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=lecroy-xstream
    T=lecroy-xstream:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    # protocol.c:439 frees TRIG_SELECT?'s answer and :441 and :446 read the source
    # from it, so stock C on this glibc opens only an answer whose source lies past
    # the 32 bytes that free writes into a chunk of that size. No scope is known to
    # answer so, as `EDGE,SR,C1,HT,OFF` has it at byte 8; pairs can come in any order
    # (the manual, TRIG_SELECT), and this form is what lets C run at all.
    S="trse=EDGE,HT,OFF,HV,1.000E-09,QL,C2,SR,C1 points=20"
    # protocol.c:580 has strcmp's sense the wrong way round: volts are amperes to C.
    # One-sided: oxrok prints no A here, and the cases named for it keep it.
    F='s/ (m?)A$/ \1V/
s/^A,A,A,A$/V,V,V,V/'
    # Both refuse at an open, each in its words. The driver's own line is compared, and
    # the words of oxrok's last line are tests.rs's.
    O='s/^oxrok: err: oxrok::drivers::lecroy_xstream: /sr: lecroy-xstream: /
s/^(Failed to open device).*$/\1./'
    UAF="protocol.c:439 frees TRIG_SELECT?'s answer, and :441 reads the source in it, where free has written (with this glibc; a preload whose free does nothing lets it through, by hand): every open fails on a scope's own EDGE,SR,C1,HT,OFF; oxrok reads it"
    UNITS="protocol.c:580 has strcmp's sense the wrong way round, so a unit of A says volts and anything else amperes: the manual's V is A to C; oxrok goes by the field"
    OFFSET="protocol.c:561 adds VERTICAL_OFFSET, and the manual subtracts it, with a worked example that only comes out so (2.44141e-7 * -1536 - 0.00054 = -0.000915): oxrok subtracts"
    ARRAYS="protocol.c:556 skips the descriptor and the user text and not TRIGTIME or RISTIME, which the manual has ahead of the data, so it gives their doubles as samples: oxrok skips them"
    RETRIES="receive_data returns TRUE on a failed read and SR_ERR, as true to a source, on a bad block, and its source stays, with nothing owed and on TCP a recv with no timeout: sigrok-cli never ends; oxrok's capture fails"
    SIGPIPE="sr_tcp_write_bytes is a plain send() and nothing ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli; oxrok's write returns EPIPE and the capture fails"
    PAST="protocol.c:553 takes wave_array_count on trust: it allocates from it and reads on past the block, and dies; oxrok refuses what doesn't fit"
    EMPTY="sr_scpi_get_block returns SR_OK and a NULL response for a block of length 0 (scpi.c:1081), and protocol.c:600 reads through it; oxrok says so"
    LIMIT="s/^(Failed to get 'limit_frames'): unknown option: .*\$/\\1: not applicable/"

    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    # What a scope answers, as far as the manual says.
    emu_case "a real trigger answer show" --oracle-bug "$UAF" --keep-header -- -d $U --show
    emu_case "a real trigger answer show tcp" --oracle-bug "$UAF" --keep-header -- -d $T --show
    emu_options points=20
    emu_case "a real trigger answer frame" --oracle-bug "$UAF" -- -d $U --frames 1 -O analog
    emu_case "a real trigger answer frame tcp" --oracle-bug "$UAF" -- -d $T --frames 2 -C CH2 -O analog

    emu_options $S
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show channel group" -- -d $T -g CH1 --show
    emu_case "get timebase" -- -d $T --get timebase
    emu_case "get vdiv" -- -d $T -g CH2 --get vdiv
    emu_case "get coupling" -- -d $T -g CH3 --get coupling
    emu_case "get num_vdiv" -- -d $T -g CH1 --get num_vdiv
    emu_case "get num_hdiv" -- -d $T --get num_hdiv
    emu_case "get samplerate" -- -d $T --get samplerate
    emu_case "get triggersource" -- -d $T --get triggersource
    emu_case "get triggerslope" -- -d $T --get triggerslope
    emu_case "get horiz_triggerpos" -- -d $T --get horiz_triggerpos
    # api.c lists it as a get and config_get has no case for it.
    emu_case "get limit_frames" --stderr --expect-fail --filter "$LIMIT" -- -d $T --get limit_frames
    emu_case "get vdiv no group" --stderr --expect-fail -- -d $T --get vdiv
    emu_case "get num_vdiv no group" --stderr --expect-fail -- -d $T --get num_vdiv

    emu_case "set timebase" --no-output -- -d $U --config timebase=2ms --set
    emu_case "set fast timebase" --no-output -- -d $T --config timebase=20ps --set
    emu_case "set vdiv" --no-output -- -d $T -g CH2 --config vdiv=500mV --set
    emu_case "set coupling" --no-output -- -d $T -g CH4 --config coupling=D50 --set
    emu_case "set triggersource" --no-output -- -d $T --config triggersource=LINE --set
    emu_case "set triggerslope" --no-output -- -d $T --config triggerslope=NEG --set
    emu_case "set slope on its source" --no-output -- -d $T --config triggersource=EXT --config triggerslope=NEG --set
    emu_case "set horiz_triggerpos" --no-output -- -d $T --config horiz_triggerpos=0.25 --set
    emu_case "set horiz_triggerpos middle" --no-output -- -d $T --config horiz_triggerpos=0.5 --set
    emu_case "set horiz_triggerpos on a timebase" --no-output -- -d $T --config timebase=50ns --config horiz_triggerpos=1 --set
    emu_case "set horiz_triggerpos too far" --no-output --expect-fail -- -d $T --config horiz_triggerpos=1.5 --set
    emu_case "set horiz_triggerpos nan" --no-output --expect-fail --oracle-differs "api.c's two comparisons let a NaN through, and it goes out as TRIG POS nan S; oxrok refuses" -- -d $T --config horiz_triggerpos=nan --set
    emu_case "set no such coupling" --no-output --expect-fail -- -d $T -g CH1 --config coupling=AC --set
    emu_case "set no such timebase" --no-output --expect-fail -- -d $T --config timebase=3ms --set
    emu_case "set vdiv no group" --no-output --expect-fail -- -d $T --config vdiv=1V --set

    emu_case "frame" --record --filter "$F" -- -d $U --frames 1 -O analog
    emu_case "frame tcp" --filter "$F" -- -d $T --frames 1 -O analog
    emu_case "two frames" --filter "$F" -- -d $T --frames 2 -O analog
    emu_case "three frames of one channel" --filter "$F" -- -d $U -C CH2 --frames 3 -O analog
    emu_case "two channels of four" --filter "$F" -- -d $T -C CH1,CH3 --frames 1 -O analog
    emu_case "a setting and a frame" --filter "$F" -- -d $T -g CH1 -c vdiv=10mV -C CH1 --frames 1 -O analog
    emu_case "samples is no limit" --stderr --expect-fail -- -d $T --samples 10 -O analog
    emu_case "time is no limit" --stderr --expect-fail -- -d $T --time 1s -O analog
    emu_case "continuous is no limit" --stderr --expect-fail -- -d $T --continuous -O analog

    # Volts as amperes, and the digits of the output.
    emu_case "volts say amperes" --oracle-differs "$UNITS" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S points=2
    emu_case "csv" --filter "s/^; (CSV generated|from) .*/; \\1/
$F" -- -d $T --frames 1 -O csv
    emu_options $S vertunit=A points=2
    emu_case "amperes say volts" --oracle-differs "$UNITS" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S vertunit=S points=2
    emu_case "another unit" --oracle-differs "$UNITS" -- -d $T -C CH1 --frames 1 -O analog

    # The descriptor.
    emu_options $S template=LECROY_2_2
    emu_case "template 2_2" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S usertext=16
    emu_case "user text" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S no-eol=1
    emu_case "no line end" --filter "$F" -- -d $T --frames 2 -O analog
    emu_options $S offset=0.25
    emu_case "vertical offset" --oracle-differs "$OFFSET" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S trigtime=32
    emu_case "sequence times" --oracle-differs "$ARRAYS" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S ristime=16
    emu_case "ris times" --oracle-differs "$ARRAYS" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog

    # No acquisition yet.
    emu_options $S empty=1
    emu_case "no data at first" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    emu_case "no data at first usb" --filter "$F" -- -d $U --frames 2 -O analog
    emu_options $S empty=3
    emu_case "no data three times" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    # Late: past what a get would wait, and a read on TCP looks on.
    emu_options $S late-at=27:100
    emu_case "a late block" --filter "$F" -- -d $T -C CH1 --frames 1 -O analog
    emu_options $S late-at=30:5
    emu_case "a late block usb" --filter "$F" -- -d $U -C CH1 --frames 1 -O analog
    emu_options $S opc-late=20
    emu_case "opc late usb" --no-output -- -d $U --config timebase=2ms --set
    # At an acquisition's pace.
    emu_options $S sweep=25
    emu_case "at a sweep's pace" --filter "$F" -- -d $T --frames 2 -O analog
    emu_case "at a sweep's pace usb" --filter "$F" -- -d $U --frames 2 -O analog

    # What is on at the start.
    emu_options $S trace=1010
    emu_case "scan with some off" --keep-header -- -d $T --scan
    emu_case "frame with some off" --filter "$F" -- -d $T --frames 1 -O analog
    emu_case "switch one on" --filter "$F" -- -d $T -C CH2 --frames 1 -O analog
    # The scope changed between the scan and the open: what the capture puts right.
    emu_options $S trace-later=1100
    emu_case "changed since the scan" --filter "$F" -- -d $T --frames 1 -O analog
    emu_options $S trace=0011 trace-later=1100
    emu_case "changed the other way" --filter "$F" -- -d $U --frames 1 -O analog
    emu_options $S trace=0000
    emu_case "all off" --stderr --expect-fail --filter 's/^.*has no enabled channels.*$/no channels/
/^Failed to start session/d' -- -d $T --frames 1 -O analog
    emu_case "all off one asked" --filter "$F" -- -d $T -C CH4 --frames 1 -O analog

    # Other answers at the open.
    emu_options vdiv=7.5E-01
    emu_case "vdiv off the table" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options $S vdiv=2E-3
    emu_case "vdiv in another form" -- -d $T -g CH1 --get vdiv
    emu_options $S tdiv=500E-9
    emu_case "timebase in another form" -- -d $T --get timebase
    emu_options $S tdiv=abc
    emu_case "timebase no number" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options $S tdiv=3.3E-3
    emu_case "timebase off the table" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options $S coupling=XYZ
    emu_case "coupling unknown" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options $S coupling=GND
    emu_case "coupling gnd" -- -d $T -g CH1 --get coupling
    emu_options $S slope=WINDOW
    emu_case "slope the driver lacks" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options $S slope=NEG
    emu_case "slope neg" -- -d $T --get triggerslope
    emu_options $S trdl=abc
    emu_case "delay no number" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options $S trdl=5.000E-01
    emu_case "delay a number" -- -d $T --get horiz_triggerpos
    emu_options $S opos=xyz
    emu_case "offset no number" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options trse=EDGE
    emu_case "trigger with no source" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options trse=EDGE,HT,OFF,HV,1.000E-09,QL,C2,SR,EX
    emu_case "source the driver lacks" --stderr --expect-fail --filter "$O" -- -d $T --show
    emu_options trse=EDGE,HT,OFF,HV,1.000E-09,QL,C2,SR,C3
    emu_case "source c3" -- -d $T --get triggersource
    emu_options $S eol=crlf
    emu_case "crlf" --keep-header -- -d $T --show

    # Silent at each kind of question of the open: the scan's five come first.
    emu_options $S mute-at=7
    emu_case "open goes silent at a vdiv" --expect-fail -- -d $U --show
    emu_options $S mute-at=9
    emu_case "open goes silent at a coupling" --expect-fail -- -d $U --show
    emu_options $S mute-at=22
    emu_case "open goes silent at the timebase" --expect-fail -- -d $U --show
    emu_options $S mute-at=23
    emu_case "open goes silent at the trigger" --expect-fail -- -d $U --show
    emu_options $S mute-at=24
    emu_case "open goes silent at the slope" --expect-fail -- -d $U --show

    # Who it is.
    emu_options idn=Acme,X,1,2
    emu_case "scan another instrument" --no-output -- -d $U --scan
    emu_case "scan another instrument tcp" --no-output -- -d $T --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $U --scan
    emu_options idn=LECROY,X
    emu_case "scan with a short idn" --no-output -- -d $U --scan
    emu_options idn=LECROY,X,3.1.4
    emu_case "scan with three fields" --keep-header -- -d $U --scan
    emu_options idn=LECROY,X,1,2
    emu_case "scan with no header" --keep-header -- -d $U --scan
    emu_options idn=LeCroy,X,1,2
    emu_case "scan in another case" --no-output -- -d $U --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options mute-at=3
    emu_case "scan goes silent at a trace" --no-output -- -d $U --scan
    emu_options late-at=1:5
    emu_case "scan late" --keep-header -- -d $U --scan
    emu_case "scan late tcp" --keep-header -- -d $T --scan
    emu_options silent-at=1
    emu_case "scan answers the second time" --no-output -- -d $U --scan

    emu_options $S opc-late=20
    emu_case "opc late" --no-output -- -d $T --config timebase=2ms --set
    emu_options $S opc-late=20
    emu_case "opc late twice over" --no-output -- -d $T -g CH1 --config coupling=A1M --set

    # What makes no capture. C goes on for ever on all but the last few.
    emu_options $S template=LECROY_9_9
    emu_case "unknown template" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "not supported" -- -d $T --frames 1 -O analog
    emu_options $S tiny=20
    emu_case "a block too short" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "too short" -- -d $T --frames 1 -O analog
    emu_options $S prefix=ALL,
    emu_case "a block after a word" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "no block" -- -d $T --frames 1 -O analog
    emu_options $S head=junk
    emu_case "no block at all" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "no block" -- -d $U --frames 1 -O analog
    emu_options $S head=#0
    emu_case "a block of no length" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "no stated length" -- -d $T --frames 1 -O analog
    emu_options $S head=#9000000000
    emu_case "a block of length 0" --no-output --oracle-bug "$EMPTY" --oxrok-fails "empty block" -- -d $T --frames 1 -O analog
    emu_options $S cut=1
    emu_case "a block cut short usb" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "came short" -- -d $U --frames 1 -O analog
    emu_options $S claim=100000000
    emu_case "more samples than the block has" --no-output --oracle-bug "$PAST" --oxrok-fails "samples from byte" -- -d $T --frames 1 -O analog
    emu_options $S comm-order=0
    emu_case "big endian" --filter "$F" --oracle-bug "$PAST" -- -d $T -C CH1 --frames 1 -O analog
    # TIME_DIV? in a capture: the open's 25 questions, the limit's *OPC?, the first
    # block, and then this one.
    emu_options $S mute-at=28
    emu_case "no answer about the timebase" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 4 --oxrok-fails "Timeout|timed out|time" -- -d $T --frames 1 -O analog
    # An instrument gone, within a frame.
    emu_options $S
    emu_case "unplug in a frame tcp" --no-output --oracle-bug "$SIGPIPE" --unplug tcp-in=27 --oxrok-fails "unplug" -- -d $T --frames 1 -O analog
    emu_case "unplug in a frame" --no-output --oracle-bug "$RETRIES" --oracle-hangs --timeout 6 --unplug in=2:27 --oxrok-fails "unplug" -- -d $U --frames 1 -O analog
    # A stop, with an acquisition that isn't coming: the open's 25, CH2 to CH4's three
    # *OPC?, the limit's, a block, TIME_DIV?, and the second frame's question.
    emu_options $S sweep=100000 mark-at=32
    emu_case "interrupt in a wait for a trigger" --oracle-bug "sigrok-cli has no SIGINT handler" --filter "$F" --interrupt tcp-model -- -d $T -C CH1 --frames 2 -O analog

    # The carrier's own.
    emu_options $S message=20
    emu_case "usbtmc message in pieces" --filter "$F" -- -d $U -C CH1 --frames 1 -O analog
    emu_options $S empty-at=1
    emu_case "usbtmc empty transfer" --keep-header -- -d $U --scan
    emu_options stall=caps
    emu_case "usbtmc capabilities stalled" --keep-header -- -d $U --scan
    emu_options bad-tag-at=1
    emu_case "usbtmc wrong tag" --no-output -- -d $U --scan
    emu_options in-stall=1
    emu_case "usbtmc bulk in stalled" --no-output -- -d $U --scan
    emu_reset
    emu_options $S
    emu_case "usbtmc by address" --keep-header -- -d lecroy-xstream:conn=usbtmc/$EMU_USB --scan
    emu_case "tcp nothing listens" --no-output -- -d lecroy-xstream:conn=tcp-raw/127.0.0.1/1 --scan
}
