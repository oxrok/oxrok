# A BeagleLogic behind its TCP server (emu/src/models/beaglelogic.rs), on a socket
# alone. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
# The C driver's other way in, /dev/beaglelogic, needs the kernel module and has no
# model: gcov's unreached lines there are all of beaglelogic_native.c.
EMU_MODEL=beaglelogic
EMU_TCP=only
EMU_DRIVER=beaglelogic
EMU_FULL=$EMU_DRIVER
EMU_GCOV=beaglelogic
EMU_GCOV_SHARED=""

emu_cases() {
    T=beaglelogic:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    C8="P8_45,P8_46,P8_43,P8_44,P8_41,P8_42,P8_39,P8_40"
    D="--samples 8 -O bits"
    # A long capture in one line and, through --digest, the hash of all of it.
    Z='$!d'
    # oxrok's log lines in C's form, one-sided.
    L='s/^oxrok: (err|warn): oxrok::drivers::beaglelogic(::protocol)?: /sr: beaglelogic: /
s/^oxrok: warn: oxrok::cli::pipeline: //'
    # An OS's words for a failed connection are its own.
    CAUSE='s/^(.*(Failed to connect to|Address lookup failed:) ?[^:]*:[^:]*): .*$/\1: CAUSE/'

    SILENT="beaglelogic_tcp_get_string's recv has no timeout (beaglelogic_tcp.c:131), and read_timeout is looked at only after it (beaglelogic_tcp.c:193), so a silent server holds sigrok-cli for ever; oxrok gives an answer 3 s"
    SIGPIPE="beaglelogic_tcp_send_cmd is a plain send (beaglelogic_tcp.c:106) and nothing ignores SIGPIPE, so the second write after the server closed kills sigrok-cli (exit 141); oxrok says the connection closed"
    SPINS="at a read of 0 beaglelogic_tcp_receive_data ends the capture and calls beaglelogic_tcp_drain, whose loop goes on while select says readable (beaglelogic_tcp.c:155-159), and a closed socket always is: sigrok-cli spins for good after its output; oxrok's capture fails"
    EMPTY="at a read of 0 beaglelogic_tcp_receive_data sends a logic packet of length 0 (protocol.c:157-161), and hex.c:163's length - unitsize wraps in an unsigned, so -O hex and -O bits crash (SIGSEGV); oxrok's capture fails"
    SIGINT="sigrok-cli has no SIGINT handler, so it dies where oxrok stops"
    RATE0="a samplerate of 0, which is what a failed read leaves, divides by zero in BUFUNIT_TIMEOUT_MS (api.c:307-308, SIGFPE at the start); oxrok refuses it at the open"
    NO_NUMBER="dev_open ignores how its gets went (api.c:172-175), and sr_atoi's failure or its 0 for an empty answer leave 0 or anything else (beaglelogic_tcp.c:225): sigrok-cli goes on with it, exit 0; the server prints only a kernel's %d, so oxrok refuses it at the open"
    LATE_OPEN="a late answer leaves 0 too, past C's one error line (beaglelogic_tcp.c:193-196, api.c:172); oxrok fails the open"
    NO_SAMPLERATE="beaglelogic_set_samplerate casts to 32 bits (beaglelogic_tcp.c:305), so a rate past them sends something else and is reported as asked; oxrok refuses it"
    ROUNDED="the module keeps a whole divisor of 100 MHz (beaglelogic_set_samplerate, kernel/beaglelogic.c:372-373), and api.c:257 keeps what was asked: sigrok-cli reports 60 MHz of a 100 MHz capture; oxrok reports what the module holds, and warns"
    CUT="a read that ends inside a 16-bit sample goes out with an odd length (protocol.c:148-161), an output drops the byte, and all that follows is a byte out; oxrok keeps the byte for the next read"
    OVERSHOOT="bytes_remaining is taken before the pre-trigger samples are added to bytes_read (protocol.c:150,167-170), so C sends a limit's worth after them: 12 of a limit of 8; oxrok counts them"
    STL_WIDTH="soft_trigger_logic_new takes its width from all channels, enabled or not (soft-trigger.c:53), so 8 bits on a 14 channel device step two bytes at a time and the trigger lands at half its place (protocol.c:168-171); oxrok uses the stream's width"
    WAIT_COUNTS="protocol.c:180 adds a read to bytes_read when the trigger has not fired, so what is thrown away counts for the limit: too few samples, or none, with exit 0; oxrok counts what it sends"
    WIDTH="dev_acquisition_start ignores a refused sampleunit (api.c:329), and C reads the module's 8 bits as 16, with exit 0; oxrok refuses the start"
    TF_WARN="dev_open ignores a refused triggerflags (api.c:180) with no word; oxrok goes on too, and warns"
    UNFIRED="C ends with nothing and says nothing; oxrok does as well, and warns"
    NO_BUFFERS="with no buffers the module's open fails and the server sends nothing (kernel/beaglelogic.c:563), and C's poll has no end; oxrok warns at the start and has a bound"
    SLOW="C says nothing of a rate whose buffer takes minutes to fill; oxrok warns at the start"

    emu_case "scan" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $T --show
    emu_case "scan 8 channels" --keep-header -- -d $T:logic_channels=8 --scan
    emu_case "scan 4 channels makes 8" --keep-header -- -d $T:logic_channels=4 --scan
    emu_case "scan 9 channels makes 14" --keep-header -- -d $T:logic_channels=9 --scan
    emu_case "show 8 channels" --keep-header -- -d $T:logic_channels=8 --show
    emu_case "scan any tcp word" --keep-header -- -d beaglelogic:conn=TCPX/127.0.0.1/$EMU_TCP_PORT --scan

    # What a scan does not take.
    emu_case "no conn and no node" --no-output -- -d beaglelogic --scan
    emu_case "not tcp" --stderr --filter "$L" -- -d beaglelogic:conn=serial/127.0.0.1/$EMU_TCP_PORT --scan
    emu_case "conn with one field" --stderr --filter "$L" -- -d beaglelogic:conn=tcp-raw --scan
    emu_case "conn with no port" --stderr --filter "$L" -- -d beaglelogic:conn=tcp-raw/127.0.0.1 --scan
    emu_case "nothing listens" --stderr --filter "$L
$CAUSE" -- -d beaglelogic:conn=tcp-raw/127.0.0.1/1 --scan
    # A port that is no number and no service: no name server is asked.
    emu_case "address lookup fails" --stderr --filter "$L
$CAUSE" -- -d beaglelogic:conn=tcp-raw/127.0.0.1/no-such-service --scan

    # What a version says.
    emu_options version=BeagleLogic_Standalone_1.0
    emu_case "the standalone board" --keep-header -- -d $T --scan
    emu_options version=beaglelogicX
    emu_case "version in any case" --keep-header -- -d $T --scan
    emu_options version=Foo
    emu_case "another device" --no-output -- -d $T --scan
    emu_options say=version:
    emu_case "an empty version" --no-output -- -d $T --scan
    emu_options late=version:100
    emu_case "a late version" --stderr --filter "$L" -- -d $T --scan
    emu_options hangup=version
    emu_case "closed at version" --no-output -- -d $T --scan
    emu_options mute=version
    emu_case "silent at version" --oracle-bug "$SILENT" --oracle-hangs --timeout 5 --stderr --filter "$L" -- -d $T --scan

    # Settings.
    emu_options
    emu_case "get samplerate" -- -d $T --get samplerate
    emu_case "get limit_samples" -- -d $T --get limit_samples
    emu_case "get captureratio" -- -d $T --get captureratio
    emu_case "get logic_channels" -- -d $T --get logic_channels
    emu_case "get logic_channels 8" -- -d $T:logic_channels=8 --get logic_channels
    emu_case "set samplerate" --no-output -- -d $T --config samplerate=1M --set
    emu_case "set captureratio" --no-output -- -d $T --config captureratio=50 --set
    emu_case "set limit_samples" --no-output -- -d $T --config limit_samples=100 --set
    emu_case "set a limit beyond the buffer" --stderr --filter "$L" -- -d $T --config limit_samples=5M --set
    emu_case "set captureratio over 100" --stderr --expect-fail --filter "$L
s/^(Failed to set device option 'captureratio': invalid argument)[.]\$/\1: it must be 0..100./" -- -d $T --config captureratio=101 --set
    emu_case "set limit 0" --stderr --expect-fail --filter "$L
s/^(Failed to set device option 'limit_samples': invalid argument)[.]\$/\1: it cannot be 0./" -- -d $T --config limit_samples=0 --set
    emu_case "set logic_channels" --stderr --expect-fail -- -d $T --config logic_channels=4 --set
    emu_case "set samplerate over the top" --stderr --expect-fail --filter "s/^(Failed to set device option 'samplerate'): generic.unspecified error[.]\$/\1: device error: The device refused 'samplerate 101000000'./" -- -d $T --config samplerate=101M --set
    emu_options refuse=samplerate
    emu_case "set samplerate refused" --stderr --expect-fail --filter "s/^(Failed to set device option 'samplerate'): generic.unspecified error[.]\$/\1: device error: The device refused 'samplerate 1000000'./" -- -d $T --config samplerate=1M --set
    emu_options refuse=triggerflags
    emu_case "open ignores a refused triggerflags" -- -d $T --get samplerate
    emu_case "open warns of a refused triggerflags" --oracle-differs "$TF_WARN" --stderr --filter "$L" -- -d $T --get samplerate
    emu_case "limit refused" --no-output --expect-fail -- -d $T --samples 8 --output-file /dev/null
    emu_options hangup=triggerflags+
    emu_case "open ignores a close at triggerflags" -- -d $T --get samplerate

    # What an open reads. Answers that are no number.
    emu_options say=samplerate:abc
    emu_case "samplerate is no number" --oracle-differs "$NO_NUMBER" --expect-fail --stderr -- -d $T --get samplerate
    emu_case "samplerate is no number capture" --oracle-bug "$RATE0" --expect-fail --stderr -- -d $T $D
    emu_options say=samplerate:
    emu_case "samplerate is empty" --oracle-differs "$NO_NUMBER" --expect-fail --stderr -- -d $T --get samplerate
    emu_options say=samplerate:-5
    emu_case "samplerate is negative" --oracle-differs "$NO_NUMBER" --expect-fail --stderr -- -d $T --get samplerate
    emu_case "samplerate is negative capture" --oracle-differs "$NO_NUMBER" --expect-fail --stderr --keep-header -- -d $T $D
    emu_options say=samplerate:0
    emu_case "samplerate is 0 capture" --oracle-bug "$RATE0" --expect-fail --stderr -- -d $T $D
    emu_options say=memalloc:abc
    emu_case "memalloc is no number" --oracle-differs "$NO_NUMBER" --expect-fail --stderr -- -d $T $D
    emu_options say=sampleunit:abc
    emu_case "sampleunit is no number" --oracle-differs "$NO_NUMBER" --expect-fail --stderr -- -d $T $D
    emu_options say=bufunitsize:abc
    emu_case "bufunitsize is no number" --oracle-differs "$NO_NUMBER" --expect-fail --stderr -- -d $T $D
    emu_options late=samplerate:100
    emu_case "samplerate late" --oracle-differs "$LATE_OPEN" --expect-fail --stderr --filter "$L" -- -d $T --get samplerate
    emu_options hangup=samplerate
    emu_case "closed at samplerate" --oracle-bug "$SIGPIPE" --expect-fail --stderr -- -d $T --get samplerate
    emu_options mute=samplerate
    emu_case "silent at samplerate" --oracle-bug "$SILENT" --oracle-hangs --timeout 5 --expect-fail --stderr -- -d $T --get samplerate
    emu_options mute=triggerflags+
    emu_case "silent at triggerflags" --oracle-bug "$SILENT" --oracle-hangs --timeout 5 -- -d $T --get samplerate

    # A capture. One-shot, as a limit makes it: the stream is the buffer's whole.
    emu_options
    emu_case "capture" --keep-header -- -d $T $D
    emu_case "capture one sample" -- -d $T --samples 1 -O bits
    emu_case "capture 8 bits" --keep-header -- -d $T -C $C8 $D
    emu_case "capture one channel past 8" --keep-header -- -d $T -C P8_20 $D
    emu_case "capture 8 channels" --keep-header -- -d $T:logic_channels=8 $D
    emu_case "capture the whole buffer" --digest --filter "$Z" -- -d $T --samples 32768 -O hex
    emu_case "capture the whole buffer 8 bits" --digest --filter "$Z" -- -d $T -C $C8 --samples 65536 -O hex
    emu_case "capture past the buffer" --stderr --filter "$L" -- -d $T --samples 32769 -O hex --output-file /dev/null
    emu_case "capture far past the buffer" --stderr --filter "$L" -- -d $T --samples 5M -O hex --output-file /dev/null
    emu_case "capture a time" --digest --filter "$Z" -- -d $T -c samplerate=1000 --time 1s -O hex
    emu_case "capture at a set rate" --keep-header -- -d $T -c samplerate=5 $D

    # What the module really holds, not what was asked.
    emu_case "a rate the module rounds" --oracle-differs "$ROUNDED" --keep-header -- -d $T -c samplerate=60M $D
    emu_case "a rate the module rounds warns" --oracle-differs "$ROUNDED" --stderr --filter "$L" -- -d $T -c samplerate=60M --samples 8 --output-file /dev/null
    emu_case "a rate past 32 bits" --oracle-differs "$NO_SAMPLERATE" --expect-fail --stderr -- -d $T -c samplerate=4294968296 $D

    # A rate whose buffer takes long is said at the start. The model doesn't wait.
    emu_options bufunitsize=4194304
    emu_case "a slow rate warns" --oracle-differs "$SLOW" --stderr --filter "$L" -- -d $T -c samplerate=100000 --samples 8 --output-file /dev/null

    # What comes after a close goes in the drain.
    emu_options tail=100
    emu_case "a tail after close" --keep-header -- -d $T $D

    # Reads that cut a stream.
    emu_options chunk=1000 memalloc=8192
    emu_case "a stream cut on samples" --digest --filter "$Z" -- -d $T --samples 4096 -O hex
    emu_options chunk=1001 memalloc=8192
    emu_case "a stream cut inside samples" --oracle-differs "$CUT" --digest --filter "$Z" -- -d $T --samples 4096 -O hex
    emu_case "a stream cut at 8 bits" --digest --filter "$Z" -- -d $T -C $C8 --samples 8192 -O hex

    # At a real pace: 40000 bytes a second, 4096 a buffer, so one every 100 ms or so,
    # and nothing before the first is full. The limit ends in the last buffer there is,
    # so that a close that comes late meets no more of them.
    emu_options paced=1 memalloc=12288
    emu_case "a time limit at the real pace" --digest --filter "$Z" -- -d $T -c samplerate=20000 --time 300ms -O hex

    # Triggers, in one read.
    emu_options
    emu_case "trigger at once" --keep-header -- -d $T $D -t P8_45=r
    emu_case "trigger in the first read" --keep-header -- -d $T $D -t P8_29=r
    emu_case "two matches" --keep-header -- -d $T $D -t P8_29=r,P8_45=0
    emu_case "level trigger" --keep-header -- -d $T $D -t P8_46=1
    emu_case "falling trigger" --keep-header -- -d $T $D -t P8_46=f
    emu_case "either edge" --keep-header -- -d $T $D -t P8_46=e
    emu_case "trigger with pre-trigger samples" --oracle-differs "$OVERSHOOT" --keep-header -- -d $T $D -c captureratio=50 -t P8_29=r
    emu_case "trigger 8 channels" --keep-header -- -d $T:logic_channels=8 $D -t P8_40=r
    emu_case "trigger 8 of 14" --oracle-differs "$STL_WIDTH" --keep-header -- -d $T -C $C8 $D -t P8_40=r
    emu_case "trigger 8 of 14 with pre-trigger samples" --oracle-differs "$STL_WIDTH" --keep-header -- -d $T -C $C8 $D -c captureratio=50 -t P8_40=r
    emu_case "trigger on a disabled channel" --stderr --expect-fail -- -d $T -C P8_45,P8_46 $D -t P8_46=r,P8_20=1
    emu_options memalloc=1024
    emu_case "a trigger that never fires" --no-output -- -d $T $D -t P8_20=1
    emu_case "a trigger that never fires warns" --oracle-differs "$UNFIRED" --stderr --filter "$L" -- -d $T $D -t P8_20=1
    # Across reads, with the stream as long as oxrok reads, as a free one's length in
    # a .tx would be the clock's.
    emu_options chunk=1024 memalloc=8192 total=3072
    emu_case "trigger later than the limit" --oracle-differs "$WAIT_COUNTS" -- -d $T --samples 100 -t P8_28=r -O bits
    emu_options chunk=1024 memalloc=8192 total=6144
    emu_case "trigger in a later read" --oracle-differs "$WAIT_COUNTS" --digest --filter "$Z" -- -d $T --samples 2000 -t P8_28=r -O hex
    emu_case "trigger in a later read with pre-trigger samples" --oracle-differs "$WAIT_COUNTS" --digest --filter "$Z" -- -d $T --samples 2000 -c captureratio=10 -t P8_28=r -O hex

    # A stop.
    emu_options
    emu_case "lib stop" --libsigrok -- -d $T --continuous --stop-after 8 -O bits
    emu_options free=1
    emu_case "lib stop on a stream" --libsigrok --digest --filter "$Z" --tx-filter 's/^tcp_in [0-9]+ -> @[0-9]+:[0-9a-f]+$/tcp_in STREAM/' -- -d $T --continuous --stop-after 8 -O bits
    # The same wire from oxrok, whose --continuous has no any-key stop.
    emu_options free=1 mark=8192
    emu_case "interrupt a stream" --oracle-bug "$SIGINT" --interrupt tcp-model --no-output --tx-filter 's/^tcp_in [0-9]+ -> @[0-9]+:[0-9a-f]+$/tcp_in STREAM/' -- -d $T --continuous --output-file /dev/null
    emu_options mark=65536
    emu_case "interrupt a capture with a limit left" --oracle-bug "$SIGINT" --interrupt tcp-model --no-output -- -d $T --samples 40000 --output-file /dev/null

    # What leaves the device silent. C waits for ever, here shown on the library, as
    # sigrok-cli can't be stopped; oxrok's bound on a stream is 30 s or more, so here it
    # is stopped, and the bound is in the unit tests.
    emu_options memalloc=0 mark=0
    emu_case "no buffers" --oracle-bug "$SIGINT" --interrupt tcp-model --stderr --filter "$L" -- -d $T --samples 8 --output-file /dev/null
    emu_case "lib no buffers" --libsigrok --hangs --timeout 3 --no-output -- -d $T $D
    emu_options first=100000 mark=0
    emu_case "silent at the stream" --oracle-bug "$SIGINT" --interrupt tcp-model --no-output -- -d $T --samples 8 --output-file /dev/null
    emu_case "lib silent at the stream" --libsigrok --hangs --timeout 3 --no-output -- -d $T $D
    emu_options mute=sampleunit+
    emu_case "silent at sampleunit" --oracle-bug "$SILENT" --oracle-hangs --timeout 5 --expect-fail --stderr -- -d $T $D
    # The module keeps 8 bits, and C reads them as 16.
    emu_options refuse=sampleunit
    emu_case "sampleunit refused" --oracle-differs "$WIDTH" --expect-fail --stderr -- -d $T $D

    # An unplug: the server closes, and what asks next finds it.
    emu_options
    emu_case "unplug after the scan" --unplug tcp-in=1 --oracle-bug "$SIGPIPE" --expect-fail --stderr -- -d $T --get samplerate
    emu_case "unplug in an open" --unplug tcp-in=3 --oracle-bug "$SIGPIPE" --expect-fail --stderr -- -d $T --get samplerate
    emu_case "unplug before the limit" --unplug tcp-in=6 --stderr --expect-fail -- -d $T $D
    # What was sent gets out first, so the limit is met, and oxrok has nothing to fail.
    emu_options mark=0
    emu_case "unplug as the limit is met" --unplug tcp-model --oracle-bug "$SPINS" --oracle-hangs --timeout 3 --no-output -- -d $T --samples 8 --output-file /dev/null
    emu_options first=100000 mark=0
    emu_case "unplug at get" --unplug tcp-model --oracle-bug "$SPINS" --oracle-hangs --timeout 3 --oxrok-fails "closed" --no-output -- -d $T --samples 8 --output-file /dev/null
    emu_options total=32768 mark=32768
    emu_case "unplug in the stream" --unplug tcp-model --oracle-bug "$EMPTY" --oxrok-fails "closed" --digest --filter "$Z" -- -d $T --samples 40000 -O hex
}
