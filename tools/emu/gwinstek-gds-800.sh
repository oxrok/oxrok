# A GW Instek GDS-800 scope (emu/src/models/gwinstek-gds-800.rs), on its serial port,
# on the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# SCPI model".
EMU_MODEL=gwinstek-gds-800
EMU_CDC=1
EMU_DRIVER=gwinstek-gds-800
EMU_FULL=$EMU_DRIVER
EMU_GCOV=gwinstek-gds-800
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    # A capture's output in few lines, and through --digest the hash of all of it.
    A='1~64!d'
    G=gwinstek-gds-800:conn=$EMU_TTY
    # Log lines in C's form; the driver's words stay as they are.
    L='s/^oxrok: err: oxrok::drivers::gwinstek_gds_800: /sr: gwinstek-gds-800: /'
    # A block refused at its head goes on arriving as a tool closes: how much is the
    # clock's. Its length goes, so that the rest of the line can still be compared.
    CUT_TX='s/^bulk_in 82 [0-9]+ -> (@|23|6a).*$/bulk_in 82 (a block, as far as the close let it)/'

    emu_case "scan" --keep-header -- -d $G --scan
    # C's group has no name, which sigrok-cli prints as a null pointer; oxrok's is empty.
    emu_case "show" --keep-header --filter 's/^    \(null\): channels/    : channels/' -- -d $G --show
    emu_case "get samplerate" -- -d $G --get samplerate
    emu_case "get limit_frames" -- -d $G --get limit_frames
    emu_case "set limit_frames" --no-output -- -d $G --config limit_frames=3 --set
    emu_case "serialcomm" --keep-header -- -d $G:serialcomm=38400/8n1 --scan

    emu_case "frame" --digest --filter "$A" -- -d $G --frames 1 -O analog
    emu_case "frame of ch1" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_case "frame of ch2" --digest --filter "$A" -- -d $G -C CH2 --frames 1 -O analog
    emu_case "two frames" --digest --filter "$A" -- -d $G --frames 2 -O analog
    emu_case "two frames of ch2" --digest --filter "$A" -- -d $G -C CH2 --frames 2 -O analog
    emu_case "frames by config" --digest --filter "$A" -- -d $G -C CH1 -c limit_frames=2 --frames 2 -O analog
    emu_case "frame keeps the header" --keep-header --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog

    # What sigrok-cli asks of a driver with frames alone.
    emu_case "samples is no limit here" --expect-fail --stderr -- -d $G --samples 10 -O analog
    emu_case "time is no limit here" --expect-fail --stderr -- -d $G --time 100 -O analog

    # Who it is.
    emu_options model=GDS-840C firmware=V.2.10
    emu_case "scan gds-840c" --keep-header -- -d $G --scan
    emu_options idn=GW,GDS-2204,P1,V.1
    emu_case "scan another gw scope" --no-output -- -d $G --scan
    emu_options idn=Rigol,GDS-820,P1,V.1
    emu_case "scan another maker" --no-output -- -d $G --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $G --scan
    emu_options idn=GW,GDS-820,V.2.03
    emu_case "scan with three fields" --keep-header -- -d $G --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $G --scan
    emu_options silent-at=1
    emu_case "scan once silent" --no-output -- -d $G --scan
    emu_options late-at=1:5
    emu_case "scan late" --keep-header -- -d $G --scan
    emu_options

    # Record lengths: each digit count, and one past what a tty buffer holds.
    emu_options points=1250
    emu_case "1250 points" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options points=5000
    emu_case "5000 points" --digest --filter "$A" -- -d $G -C CH2 --frames 1 -O analog
    emu_options points=50000
    emu_case "50000 points" --digest --filter '1~4096!d' -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # What it says of its scale.
    emu_options scale1=100mV scale2=2V
    emu_case "scale in mv and v" --digest --filter "$A" -- -d $G --frames 1 -O analog
    emu_options scale1=2.000E-03 scale2=5.000E+00
    emu_case "scale in nr3" --digest --filter "$A" -- -d $G --frames 1 -O analog
    emu_options

    # Pieces, and a pause before a block.
    emu_options chunk=32
    emu_case "block in pieces" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options chunk=1
    emu_case "block byte by byte" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options late-at=2:10
    emu_case "block a while late" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options late-at=3:5
    emu_case "scale a while late" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # The line, set and through a reopen.
    emu_options uart=38400/8n1
    emu_case "behind a uart" --digest --filter "$A" -- -d $G:serialcomm=38400/8n1 -C CH1 --frames 1 -O analog
    emu_case "behind a uart it isn't set to" --no-output -- -d $G --scan
    # C doesn't look at the channel byte, and oxrok warns.
    emu_options indicator=9
    emu_case "another channel's indicator" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # A silent or wrong answer.
    STRTOD="protocol.c:226 takes g_ascii_strtod's result with no check, so what isn't a number is 0 V, every sample -0.0 V, with exit 0; oxrok's capture fails"
    MV="protocol.c:227 compares the tail with mV alone, so 500mv is 500 V; oxrok's capture fails"
    NO_STOP="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    NEVER="the read state of protocol.c:157 and those after it have no bound, so a silent scope holds C for ever: the libsigrok case with this one's name pins it"
    GONE="protocol.c:92-93 returns at once on any revents but G_IO_IN or 0, and a gone tty's has G_IO_HUP too, so C goes round for ever and no read reports it (gcov reaches line 93 in this case); oxrok's read fails"
    emu_options silent-at=3
    emu_case "scale silent" --oxrok-fails "No answer to :CHAN1:SCAL?" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options late-at=3:150
    emu_case "scale too late" --oxrok-fails "No answer to :CHAN1:SCAL?" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options scale1=abc
    emu_case "scale no number" --oxrok-fails "no scale" --oracle-differs "$STRTOD" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options scale1=0V
    emu_case "scale of nothing" --oxrok-fails "no scale" --oracle-differs "$STRTOD" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options scale1=500mv
    emu_case "scale in lower case mv" --oxrok-fails "no scale" --oracle-differs "$MV" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog

    # A line feed in a block. scpi_serial.c has kept line ends since 9ea4018f (2017),
    # and the driver still puts back the one it thinks was stripped.
    LF_END="the manual's words on all answers: protocol.c's next get of :CHAN1:SCAL? reads the line feed as its whole answer, and 0 V: every sample -0.0 V, with exit 0, where oxrok reads past an empty line. Not what a working scope can have sent, see the model"
    LF_LAST="read_data (protocol.c:46-49) adds a line feed after a short read ending in one, so the last read comes to one byte too many: Too many bytes read, no samples, exit 0. oxrok takes a point of 0x0A as a point"
    SIGN="sr_atoi takes a sign, so C reads a block of 100 bytes and the rest as its scale, and 0 V; oxrok refuses what is no size"
    SMALL="C's size below its 8 fields asks for a read of a negative size (protocol.c:215)"
    OVER="MAX_SAMPLES and rcv_buffer hold 125000 points and no check looks: C writes past them; oxrok refuses"
    emu_options wave=full points=500
    emu_case "every low byte" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options wave=full points=2500 chunk=1
    # Pieces of one byte make C's reads end on a line feed nearly always, but not always:
    # 14 runs of 15 gave -0.0 V and one the right values. So this holds oxrok alone, and
    # "every low byte" above holds C's agreement where no read ends so.
    emu_case "line feeds in a block" --no-oracle-only --filter 1q -- -d $G -C CH1 --frames 1 -O analog
    emu_options end=lf
    emu_case "line feed behind each block" --oracle-differs "$LF_END" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options points=5000 last=0a
    emu_case "line feed at a block's end" --oracle-differs "$LF_LAST" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_case "lib line feed at a block's end" --libsigrok --stderr -- -d $G -C CH1 --frames 1 -O analog

    # The head.
    emu_options head=#3100
    emu_case "size digit 3" --oxrok-fails "is not 4, 5 or 6" --stderr --filter "$L" --tx-filter "$CUT_TX" -- -d $G -C CH1 --frames 1 -O analog
    emu_options head=#41a08
    emu_case "size no number" --oxrok-fails "Could not parse" --stderr --filter "$L" --tx-filter "$CUT_TX" -- -d $G -C CH1 --frames 1 -O analog
    emu_options head=#4+100
    emu_case "size with a sign" --oxrok-fails "Could not parse" --oracle-differs "$SIGN" --stderr --filter "$L" --tx-filter "$CUT_TX" -- -d $G -C CH1 --frames 1 -O analog
    emu_options head=#40004
    emu_case "size under its fields" --oxrok-fails "out of range" --oracle-differs "$SMALL" --stderr --filter "$L" --tx-filter "$CUT_TX" -- -d $G -C CH1 --frames 1 -O analog
    emu_options head=#40008 cut=8
    emu_case "size of no points" --no-output -- -d $G -C CH1 --frames 1 -O analog
    emu_options points=125000
    emu_case "the largest record" --digest --filter '1~8192!d' -- -d $G -C CH1 --frames 1 -O analog
    emu_options points=125001
    emu_case "one point past the largest" --oxrok-fails "out of range" --oracle-differs "$OVER" --digest --filter "$A" --tx-filter "$CUT_TX" -- -d $G -C CH1 --frames 1 -O analog
    emu_options points=130000
    emu_case "past the largest" --oxrok-fails "out of range" --oracle-differs "$OVER" --stderr --filter "$L" --tx-filter "$CUT_TX" -- -d $G -C CH1 --frames 1 -O analog
    emu_options head=junk#41008
    emu_case "junk ahead of a block" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # A silence holds C for ever. The end of oxrok's wait is 10 minutes off, and a unit
    # test's: here an interrupt on oxrok, and a library run for C's hang.
    emu_options mute-at=2 mark-at=2
    emu_case "block silent" --no-output --oracle-bug "$NO_STOP: $NEVER" --interrupt model --timeout 8 -- -d $G -C CH1 --frames 1 -O analog
    emu_options mute-at=2
    emu_case "lib block silent" --libsigrok --hangs --timeout 3 --no-output -- -d $G -C CH1 --frames 1 -O analog
    emu_options firmware=V.2.01 mark-at=2
    emu_case "firmware before the memory read" --no-output --oracle-bug "$NO_STOP: $NEVER" --interrupt model --timeout 8 -- -d $G -C CH1 --frames 1 -O analog
    emu_options cut=100 mark-at=2
    emu_case "block cut short" --no-output --oracle-bug "$NO_STOP: $NEVER" --interrupt model --timeout 8 -- -d $G -C CH1 --frames 1 -O analog
    emu_options cut=100
    emu_case "lib block cut short" --libsigrok --hangs --timeout 3 --no-output -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # An unplug.
    emu_options mute-at=2 mark-at=2
    emu_case "unplug waiting for a block" --no-output --oracle-bug "$GONE" --oracle-hangs --oxrok-fails "gone" --unplug model --timeout 4 --tx-filter '/^unplug$/q' -- -d $G -C CH1 --frames 1 -O analog
    emu_options mute-at=3 mark-at=3
    emu_case "unplug waiting for a scale" --no-output --oxrok-fails "gone" --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # With no limit it goes on until stopped: limit_frames=0 wraps in C.
    emu_options mute-at=6 mark-at=6
    emu_case "no frame limit" --digest --filter "$A" --oracle-bug "$NO_STOP" --interrupt model --timeout 8 -- -d $G -C CH1 --frames 0 -O analog
    emu_options
}
