# Siglent SDS scopes (emu/src/models/siglent-sds.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
#
# Stock C ends no capture with samples but one that a read error ends (docs/DEVICES.md),
# so most of what a capture does is pinned on oxrok alone: what C does there stays in
# each case's .oracle. What C can run is compared in full: every setting, and a capture
# of waveforms with no samples, whose wire is all there. A USBTMC capture costs C 7 s a
# channel, its source's timeout with nothing to read. An E series on the screen goes as
# libsigrok pull request 247 does, so its captures are oxrok's own wire throughout.
EMU_MODEL=siglent-sds
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=siglent-sds
EMU_FULL=$EMU_DRIVER
EMU_GCOV=siglent-sds
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    A='1~64!d'
    U=siglent-sds
    T=siglent-sds:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    FEW="under 363 samples, the first read has more than they (protocol.c:562): it moves devc->buffer for good by the descriptor (:565) and sends the same count again, past the buffer's end to a segfault; oxrok reads them"
    ZERO="protocol.c:569 asks for num_samples - num_block_bytes, which has counted the 363 of the header, and :612 ends only at num_samples + 363 of them, so once it holds num_samples bytes it asks for 0 for ever, sending an empty packet each time, having sent the samples from the third to the 361st before the last; on this box the process then dies at a heap check of glibc's, and pull request 247's report shows the empty packets going on; oxrok reads the block to its end"
    BEGIN_TWICE="channel_start (protocol.c:298) begins a read of the answer and the callback (:531) begins another, whose request nothing answers: LIBUSB_ERROR_TIMEOUT, and each next callback asks again, for ever (bug 1356's title); oxrok asks once"
    NO_END="a block with no samples is the one C ends, by read_complete, which a raw TCP read of exactly 363 bytes never says (scpi_tcp.c:171): 'Read should have been completed', and the capture ends at its first channel"
    NO_FPAR="capture_start reads FPAR? with no read begun (protocol.c:192), which on USBTMC is a read past the last message: 'Read error while reading data header', and the start fails; oxrok begins one"
    E_SINGLE="for an E series oxrok goes as libsigrok pull request 247, the one code shown to finish captures on real ones: TRMD SINGLE, TRMD? until STOP, the scales again, then C1:WF? DAT2, where C sends ARM, :INR? and C1:WF? ALL; a block with no samples C ends at its first channel ('Read should have been completed', scpi_tcp.c:171)"
    INR0="after ARM, protocol.c:174-183 takes anything but 8192 and 8193 for a scope that did not arm and fails the start; the guide gives 0 as single mode with no signal yet (p.175), and oxrok polls on"
    SLOPE="after a set, api.c:558 asks '%s:TRSL' of the list's name, CH2, where the open asked of the scope's, C2: C sends CH2:TRSL NEG"
    KPTS="protocol.c:916 takes Kpts by 10000, ten times what it says; oxrok by 1000"
    PTS="protocol.c:918 makes 0 of a number with neither Mpts nor Kpts, so a rate of 0; oxrok reads it"
    PLAIN="protocol.c:918 makes 0 of a plain number, the form the guide gives for the older models (p.35), so a rate of 0; oxrok reads it"
    VDIV5="api.c:109 has { 500, 100000 } for 500 uV, which is 5 mV: C says that of 5 mV, and C's first entry of a --show says 0.005 V where it sends 500UV"
    CUT="C's first 363 bytes hold two samples it drops (protocol.c:565 is on the other path), and it reads to the message's end, a read error, which it takes for an end at exit 0; oxrok has all that came, fails, and says so. tests/siglent_sds_oracle.rs compares the two by value"
    FOUR="protocol.c:802 does a strlen of the fifth field after checking for four: SIGSEGV"
    DISW="api.c:833 sends DI:SW? where its comment says to turn the analyzer on, reads no answer and takes it as on. On this model the answer goes when the next command comes, as IEEE 488.2 has it, so the analyzer stays off and C reads it anyway; a scope that sends at once would have the next query read it. It takes 15 bytes off a head of 16 as well. oxrok sends DI:SW ON"
    DIGITAL="get_digital reads with a maxlen of -1 (protocol.c:375), which on TCP is recv of everything there is, and takes 15 bytes off a head of 16; it stops the capture at its first logic packet, twice (protocol.c:660, :664)"
    DIGITAL_USB="channel_start begins a read with nothing asked for a logic channel (protocol.c:298), which USBTMC waits out and fails, for ever"
    NEVER_ENDS="siglent_sds_receive returns TRUE on each failed step and its source stays (protocol.c:494, :500), so with the device gone it tries again for ever; oxrok's capture fails"
    PIECES="siglent_sds_read_header wants 363 bytes of one read (protocol.c:323), and USBTMC gives one transfer's: 'Read error while reading data header', which ends the capture with nothing; oxrok reads what comes"
    SIGPIPE="sr_tcp_write_bytes is a plain send() (tcp.c) and nothing in libsigrok or sigrok-cli ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli (exit 141) in the middle of its output; oxrok's write returns EPIPE and the capture fails"
    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block (nonblocking FALSE, scpi_tcp.c), and its recv() has no timeout, so sr_scpi_get_string's 1 s (scpi.c) never runs and a silent instrument holds sigrok-cli for ever; oxrok's read has the timeout"

    # An SDS1104X-E, the default.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $T --show
    emu_case "show usb" --keep-header -- -d $U --show
    emu_case "show channel group" --oracle-differs "$VDIV5" -- -d $T -g CH1 --show
    emu_case "show la group" -- -d $T -g LA --show
    for key in timebase triggersource triggerslope triggerlevel horiz_triggerpos num_hdiv samplerate limit_frames data_source averaging avg_samples; do
        emu_case "get $key" -- -d $T --get $key
    done
    for key in vdiv coupling probe_factor num_vdiv; do
        emu_case "get $key" -- -d $T -g CH2 --get $key
    done
    emu_case "set timebase" --no-output -- -d $U --config timebase=2ms --set
    emu_case "set timebase in ns" --no-output -- -d $T --config timebase=5ns --set
    emu_case "set trigger source" --no-output -- -d $T --config triggersource=CH2 --set
    emu_case "set trigger source ext" --no-output -- -d $T "--config=triggersource=Ext /5" --set
    emu_case "set trigger source line" --no-output -- -d $T "--config=triggersource=AC Line" --set
    emu_case "set trigger slope" --no-output -- -d $T --config triggerslope=f --set
    emu_case "set a source and its slope" --no-output --oracle-differs "$SLOPE" -- -d $T --config triggersource=CH2 --config triggerslope=f --set
    emu_case "set trigger level" --no-output -- -d $T --config triggerlevel=0.5 --set
    emu_case "set horiz_triggerpos" --no-output -- -d $T --config horiz_triggerpos=0.25 --set
    emu_case "set vdiv" --no-output -- -d $T -g CH2 --config vdiv=500mV --set
    emu_case "set coupling" --no-output -- -d $T -g CH2 "--config=coupling=D50 DC 50 Ohm" --set
    emu_case "set probe factor" --no-output -- -d $T -g CH1 --config probe_factor=10 --set
    emu_case "set data source" --no-output -- -d $T --config data_source=History --set
    emu_case "set no such timebase" --no-output --expect-fail -- -d $T --config timebase=3ms --set
    emu_case "set no such vdiv" --no-output --expect-fail -- -d $T -g CH1 --config vdiv=3mV --set
    emu_case "set no such source" --no-output --expect-fail -- -d $T --config triggersource=CH9 --set
    emu_case "set no such position" --no-output --expect-fail -- -d $T --config horiz_triggerpos=2 --set

    # It has frames for a limit and nothing else, on either tool.
    emu_case "no time limit" --no-output --expect-fail -- -d $T --time 100 -O analog
    emu_case "no sample limit" --no-output --expect-fail -- -d $T --samples 10 -O analog

    # How a USBTMC device is found and held.
    emu_reset
    emu_case "usbtmc by address" --keep-header -- -d siglent-sds:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header -- -d siglent-sds:conn=usbtmc --scan
    emu_case "tcp nothing listens" --no-output -- -d siglent-sds:conn=tcp-raw/127.0.0.1/1 --scan

    # Captures on an E series.
    emu_case "frame" --oracle-bug "$BEGIN_TWICE" --oracle-hangs --timeout 10 --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O analog
    emu_case "frame tcp" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_case "two frames of two channels tcp" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T -C CH1,CH3 --frames 2 -O analog
    emu_options trse=EDGE,SR,C1,HT
    emu_case "four fields of trse" --oracle-bug "$FOUR" -- -d $T --get triggerslope
    emu_options trse=EDGE,SR,C1
    emu_case "three fields of trse" --no-output --expect-fail -- -d $T --get triggerslope
    emu_options trse=EDGE,SR,EX,HT,OFF
    emu_case "a source with no level" -- -d $T --get triggerlevel
    emu_case "a source of the scope" -- -d $T --get triggersource
    emu_options trse=EDGE,SR,C1,HT,200US
    emu_case "a position as C reads it" -- -d $T --get horiz_triggerpos
    emu_options off=2,4
    emu_case "channels off show" -- -d $T -g CH1 --get coupling
    emu_case "channels off capture what is on tcp" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T --frames 1 -O analog
    emu_options off=1 trmd=3
    emu_case "waits for a single shot tcp" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options depth=0
    emu_case "e frame of nothing tcp" --no-output --oracle-differs "$E_SINGLE" -- -d $T -C CH1,CH2 --frames 1 -O analog
    emu_options depth=32 la=1
    emu_case "digital channels tcp" --oracle-differs "$DIGITAL" -- -d $T -C D0,D9 --frames 1 -O bits
    emu_case "digital channels" --oracle-bug "$DIGITAL_USB" --oracle-hangs --timeout 10 -- -d $U -C D0,D9 --frames 1 -O bits
    emu_case "analog and digital tcp" --oracle-bug "$FEW" --digest --filter "$A" -- -d $T -C CH1,D0 --frames 1 -O analog
    emu_case "analog with the analyzer on tcp" --oracle-bug "$FEW" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options depth=32
    emu_case "digital turns the analyzer on tcp" --oracle-differs "$DISW" -- -d $T -C D0 --frames 1 -O bits
    # What C sent, which an --oracle-differs case doesn't keep.
    emu_case "lib digital with the analyzer off tcp" --libsigrok -- -d $T -C D0 --frames 1 -O bits

    # An SPO model, the one whose USBTMC reads C gets through.
    emu_options model=SDS1202X sanu=1.40Mpts depth=400
    emu_case "spo show" --keep-header -- -d $T --show
    emu_case "spo get samplerate" -- -d $T --get samplerate
    emu_options model=SDS1202X sanu=70.00Kpts
    emu_case "spo samplerate in kpts" --oracle-differs "$KPTS" -- -d $T --get samplerate
    emu_options model=SDS1202X sanu=700pts
    emu_case "spo samplerate in pts" --oracle-differs "$PTS" -- -d $T --get samplerate
    emu_options model=SDS1202X depth=0
    emu_case "spo empty frame" --record --no-output -- -d $U --frames 1 -O analog
    emu_case "spo empty frames" --no-output -- -d $U -C CH1,CH2 --frames 2 -O analog
    emu_case "spo averaging" --no-output -- -d $U -c averaging=1:avg_samples=16 --frames 1 -O analog
    emu_case "spo empty frame tcp" --no-output --oracle-differs "$NO_END" -- -d $T --frames 1 -O analog
    emu_options model=SDS1202X depth=0 inr=8192,0,0,1
    emu_case "spo waits for a trigger" --no-output -- -d $U --frames 1 -O analog
    emu_options model=SDS1202X depth=0 inr=0,0,1
    emu_case "spo inr of 0 after arm" --no-output --oracle-bug "$INR0" -- -d $U --frames 1 -O analog
    emu_options model=SDS1202X depth=0
    emu_case "lib a source and its slope" --no-output --libsigrok -- -d $U -c triggersource=CH2 -c triggerslope=f --frames 1 -O analog
    emu_case "lib horiz_triggerpos" --no-output --libsigrok -- -d $U -c horiz_triggerpos=0.25 --frames 1 -O analog
    emu_options model=SDS1202X depth=0
    emu_case "spo history" --no-output --oracle-bug "$NO_FPAR" -- -d $U -c data_source=History --frames 1 -O analog
    emu_options model=SDS1202X depth=400
    emu_case "spo frame" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O analog
    emu_case "spo frame tcp" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T -C CH2 --frames 1 -O analog
    emu_case "spo history tcp" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T -C CH1 -c data_source=History --frames 2 -O analog
    emu_case "usbtmc kernel driver bound" --oracle-bug "$ZERO" --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O analog
    emu_options model=SDS1202X depth=400 cut=363
    emu_case "spo cut" --oracle-differs "$CUT" --oxrok-fails "was cut short" -- -d $U -g CH1 -c vdiv=10mV -C CH1 --frames 1 -O analog
    emu_options model=SDS1202X depth=400 message=20
    emu_case "usbtmc message in pieces" --oracle-differs "$PIECES" --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O analog

    # A non-SPO model: no ARM.
    emu_options model=SDS1052DL+ sanu=1.40Mpts
    emu_case "non spo show" --keep-header -- -d $T --show
    emu_options model=SDS1052DL+ depth=1600
    emu_case "non spo samplerate as the guide has it" --oracle-differs "$PLAIN" -- -d $T --get samplerate
    emu_options model=SDS1052DL+ depth=0 inr=0,0,8193
    emu_case "non spo empty frame" --no-output -- -d $U --frames 1 -O analog
    emu_options model=SDS1052DL+ depth=400
    emu_case "non spo frame" --oracle-bug "$ZERO" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options model=SDS2304X sanu=1.40Mpts
    emu_case "four channels of an spo" --keep-header -- -d $T --scan
    emu_options model=SDS1202X-E
    emu_case "an e series of two" --keep-header -- -d $T --show

    # What answers, and when.
    emu_options idn=Acme,X,1,2
    emu_case "scan another instrument" --no-output -- -d $U --scan
    emu_options idn=Acme,SDS1202X,1,2
    emu_case "scan by the model alone" --keep-header -- -d $T --scan
    emu_options idn=Siglent,SDS1202X,7.1
    emu_case "scan with no serial number" --keep-header -- -d $T --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $T --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_case "scan silent tcp" --no-output --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --scan
    emu_options silent-at=1
    emu_case "scan answers the second time" --keep-header -- -d $U --scan
    emu_options mute-at=3
    emu_case "show goes silent" --expect-fail -- -d $U --show
    emu_options late-at=1:5
    emu_case "scan late" --keep-header -- -d $U --scan
    emu_case "scan late tcp" --keep-header -- -d $T --scan
    emu_options stall=caps
    emu_case "usbtmc capabilities stalled" --keep-header -- -d $U --scan
    emu_options bad-tag-at=1
    emu_case "usbtmc wrong tag" --keep-header -- -d $U --scan

    # An unplug. On TCP the instrument closes the connection.
    emu_options
    # After the readback, so that the capture's first write comes next.
    emu_case "unplug before a capture tcp" --digest --filter "$A" --oracle-bug "$SIGPIPE" --unplug tcp-in=26 --oxrok-fails "unplug" -- -d $T -C CH1 --frames 2 -O analog
    # After the first channel's waveform: C has gone by then (ZERO), and oxrok's next write or read finds it.
    emu_case "unplug in a frame tcp" --digest --filter "$A" --oracle-bug "$ZERO" --unplug tcp-in=32 --oxrok-fails "unplug" -- -d $T -C CH1,CH2 --frames 2 -O analog
    # After :INR?, C goes on asking for ever.
    emu_options model=SDS1202X depth=400
    emu_case "unplug in a frame" --digest --filter "$A" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 12 --unplug in=2:20 --oxrok-fails "unplug" -- -d $U -C CH1 --frames 2 -O analog
}
