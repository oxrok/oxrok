# A BeagleLogic behind its TCP server (emu/src/models/beaglelogic.rs), on a socket
# alone. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
# The C driver's other way in, /dev/beaglelogic, needs the kernel module and has no
# model: gcov's unreached lines there are all of beaglelogic_native.c.
EMU_MODEL=beaglelogic
EMU_TCP=only
EMU_DRIVER=beaglelogic
EMU_GCOV=beaglelogic
EMU_GCOV_SHARED=""

emu_cases() {
    T=beaglelogic:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    C8="P8_45,P8_46,P8_43,P8_44,P8_41,P8_42,P8_39,P8_40"
    D="--samples 8 -O bits"
    # A long capture in one line and, through --digest, the hash of all of it.
    Z='$!d'
    # An OS's words for a refused connection are its own.
    REFUSED='s/^(.*Failed to connect to [^:]*:[0-9]*): .*$/\1: CAUSE/'

    emu_case "scan" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $T --show
    emu_case "scan 8 channels" --keep-header -- -d $T:logic_channels=8 --scan
    emu_case "scan 4 channels makes 8" --keep-header -- -d $T:logic_channels=4 --scan
    emu_case "scan 9 channels makes 14" --keep-header -- -d $T:logic_channels=9 --scan
    emu_case "show 8 channels" --keep-header -- -d $T:logic_channels=8 --show
    emu_case "scan any tcp word" --keep-header -- -d beaglelogic:conn=TCPX/127.0.0.1/$EMU_TCP_PORT --scan

    # What a scan does not take.
    emu_case "no conn and no node" --no-output -- -d beaglelogic --scan
    emu_case "not tcp" --stderr -- -d beaglelogic:conn=serial/127.0.0.1/$EMU_TCP_PORT --scan
    emu_case "conn with one field" --stderr -- -d beaglelogic:conn=tcp-raw --scan
    emu_case "conn with no port" --stderr -- -d beaglelogic:conn=tcp-raw/127.0.0.1 --scan
    emu_case "nothing listens" --stderr --filter "$REFUSED" -- -d beaglelogic:conn=tcp-raw/127.0.0.1/1 --scan

    # A port that is no number and no service: no name server is asked.
    LOOKUP='s/^(.*Address lookup failed: [^:]*:[^:]*): .*$/\1: CAUSE/'
    emu_case "address lookup fails" --stderr --filter "$LOOKUP" -- -d beaglelogic:conn=tcp-raw/127.0.0.1/no-such-service --scan

    # What a version says.
    emu_options version=BeagleLogic_Standalone_1.0
    emu_case "the standalone board" --keep-header -- -d $T --scan
    emu_options version=beaglelogicX
    emu_case "version in any case" --keep-header -- -d $T --scan
    emu_options version=Foo
    emu_case "another device" --no-output -- -d $T --scan
    emu_options say=version:
    emu_case "an empty version" --no-output -- -d $T --scan
    emu_options late=version:75
    emu_case "a late version" --stderr -- -d $T --scan
    emu_options hangup=version
    emu_case "closed at version" --no-output -- -d $T --scan
    emu_options mute=version
    emu_case "silent at version" --hangs --timeout 3 --no-output -- -d $T --scan

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
    emu_case "set a limit beyond the buffer" --stderr -- -d $T --config limit_samples=5M --set
    emu_case "set captureratio over 100" --stderr --expect-fail -- -d $T --config captureratio=101 --set
    emu_case "set limit 0" --stderr --expect-fail -- -d $T --config limit_samples=0 --set
    emu_case "set logic_channels" --stderr --expect-fail -- -d $T --config logic_channels=4 --set
    emu_case "set samplerate over the top" --stderr --expect-fail -- -d $T --config samplerate=101M --set
    emu_options refuse=samplerate
    emu_case "set samplerate refused" --stderr --expect-fail -- -d $T --config samplerate=1M --set
    emu_options refuse=triggerflags
    emu_case "open ignores a refused triggerflags" -- -d $T --get samplerate
    emu_case "limit refused" --stderr --expect-fail -- -d $T $D
    emu_options hangup=triggerflags+
    emu_case "open ignores a close at triggerflags" -- -d $T --get samplerate

    # What an open reads. Answers that are no number.
    emu_options say=samplerate:abc
    emu_case "samplerate is no number" -- -d $T --get samplerate
    emu_case "samplerate is no number capture" --expect-fail --no-output -- -d $T $D
    emu_options say=samplerate:
    emu_case "samplerate is empty" -- -d $T --get samplerate
    emu_options say=samplerate:-5
    emu_case "samplerate is negative" -- -d $T --get samplerate
    emu_case "samplerate is negative capture" --keep-header -- -d $T $D
    emu_options say=samplerate:0
    emu_case "samplerate is 0 capture" --expect-fail --no-output -- -d $T $D
    emu_options say=memalloc:abc
    emu_case "memalloc is no number" --stderr -- -d $T $D
    emu_options say=sampleunit:abc
    emu_case "sampleunit is no number" -- -d $T $D
    emu_options say=bufunitsize:abc
    emu_case "bufunitsize is no number" -- -d $T $D
    emu_options late=samplerate:75
    emu_case "samplerate late" --stderr -- -d $T --get samplerate
    emu_options hangup=samplerate
    emu_case "closed at samplerate" --expect-fail --no-output -- -d $T --get samplerate
    emu_options mute=samplerate
    emu_case "silent at samplerate" --hangs --timeout 3 --no-output -- -d $T --get samplerate
    emu_options mute=triggerflags+
    emu_case "silent at triggerflags" --hangs --timeout 3 --no-output -- -d $T --get samplerate

    # A capture. One-shot, as a limit makes it: the stream is the buffer's whole.
    emu_options
    emu_case "capture" --keep-header -- -d $T $D
    emu_case "capture one sample" -- -d $T --samples 1 -O bits
    emu_case "capture 8 bits" --keep-header -- -d $T -C $C8 $D
    emu_case "capture one channel past 8" --keep-header -- -d $T -C P8_20 $D
    emu_case "capture 8 channels" --keep-header -- -d $T:logic_channels=8 $D
    emu_case "capture the whole buffer" --digest --filter "$Z" -- -d $T --samples 32768 -O hex
    emu_case "capture the whole buffer 8 bits" --digest --filter "$Z" -- -d $T -C $C8 --samples 65536 -O hex
    emu_case "capture past the buffer" --stderr -- -d $T --samples 32769 -O hex --output-file /dev/null
    emu_case "capture far past the buffer" --stderr -- -d $T --samples 5M -O hex --output-file /dev/null
    emu_case "capture a time" --digest --filter "$Z" -- -d $T -c samplerate=1000 --time 1s -O hex
    emu_case "capture at a set rate" --keep-header -- -d $T -c samplerate=5 $D
    # sigrok-cli's --continuous stops at its stdin's end, here at once.
    emu_case "capture continuous" --no-output -- -d $T --continuous -O hex

    # At what the module really holds, not what was asked.
    emu_case "a rate the module rounds" --keep-header -- -d $T -c samplerate=60M $D
    # (uint32_t) in beaglelogic_set_samplerate: 1000 goes out.
    emu_case "a rate past 32 bits" --keep-header -- -d $T -c samplerate=4294968296 $D

    # What comes after a close goes in the drain.
    emu_options tail=100
    emu_case "a tail after close" --keep-header -- -d $T $D

    # Reads that cut a stream.
    emu_options chunk=1000 memalloc=8192
    emu_case "a stream cut on samples" --digest --filter "$Z" -- -d $T --samples 4096 -O hex
    emu_options chunk=1001 memalloc=8192
    emu_case "a stream cut inside samples" --digest --stderr -- -d $T --samples 4096 -O hex
    emu_case "a stream cut at 8 bits" --digest --filter "$Z" -- -d $T -C $C8 --samples 8192 -O hex

    # At a real pace: 40000 bytes a second, 4096 a buffer, so one every 100 ms or so,
    # and nothing before the first is full.
    emu_options paced=1 memalloc=16384
    emu_case "a time limit at the real pace" --digest --filter "$Z" -- -d $T -c samplerate=20000 --time 300ms -O hex

    # Triggers, in one read.
    emu_options
    emu_case "trigger at once" --keep-header -- -d $T $D -t P8_45=r
    emu_case "trigger in the first read" --keep-header -- -d $T $D -t P8_29=r
    emu_case "two matches" --keep-header -- -d $T $D -t P8_29=r,P8_45=0
    emu_case "level trigger" --keep-header -- -d $T $D -t P8_46=1
    emu_case "falling trigger" --keep-header -- -d $T $D -t P8_46=f
    emu_case "either edge" --keep-header -- -d $T $D -t P8_46=e
    emu_case "trigger with pre-trigger samples" --keep-header -- -d $T $D -c captureratio=50 -t P8_29=r
    emu_case "trigger 8 channels" --keep-header -- -d $T:logic_channels=8 $D -t P8_40=r
    emu_case "trigger 8 of 14" --keep-header -- -d $T -C $C8 $D -t P8_40=r
    emu_case "trigger 8 of 14 with pre-trigger samples" --keep-header -- -d $T -C $C8 $D -c captureratio=50 -t P8_40=r
    emu_case "trigger on a disabled channel" --stderr --expect-fail -- -d $T -C P8_45,P8_46 $D -t P8_46=r,P8_20=1
    emu_options memalloc=1024
    emu_case "a trigger that never fires" --no-output -- -d $T $D -t P8_20=1
    # Across reads.
    emu_options chunk=1024 memalloc=8192
    emu_case "trigger later than the limit" --no-output -- -d $T --samples 100 -t P8_28=r -O bits
    emu_case "trigger in a later read" --digest --stderr -- -d $T --samples 2000 -t P8_28=r -O hex
    emu_case "trigger in a later read with pre-trigger samples" --digest --stderr -- -d $T --samples 2000 -c captureratio=10 -t P8_28=r -O hex

    # A stop.
    emu_options
    emu_case "lib stop" --libsigrok -- -d $T --continuous --stop-after 8 -O bits
    emu_options free=1
    emu_case "lib stop on a stream" --libsigrok --digest --filter "$Z" --tx-filter 's/^tcp_in [0-9]+ -> @[0-9]+:[0-9a-f]+$/tcp_in STREAM/' -- -d $T --continuous --stop-after 8 -O bits

    # What leaves the device silent.
    emu_options memalloc=0
    emu_case "no buffers" --hangs --timeout 3 --stderr -- -d $T $D
    emu_options first=1000
    emu_case "silent at the stream" --hangs --timeout 3 --no-output -- -d $T $D
    emu_options mute=sampleunit+
    emu_case "silent at sampleunit" --hangs --timeout 3 --no-output -- -d $T $D
    # The module keeps 8 bits, and C reads them as 16.
    emu_options refuse=sampleunit
    emu_case "sampleunit refused" --keep-header -- -d $T $D

    # An unplug: the server closes.
    emu_options
    emu_case "unplug in an open" --unplug tcp-in=3 --expect-fail --no-output -- -d $T --get samplerate
    emu_case "unplug before the limit" --unplug tcp-in=6 --stderr --expect-fail -- -d $T $D
    emu_options mark=0
    emu_case "unplug at get" --unplug tcp-model --hangs --timeout 3 --no-output -- -d $T --samples 8 -O bits --output-file /dev/null
    emu_options mark=65536
    emu_case "unplug in the stream" --unplug tcp-model --hangs --timeout 3 --stderr -- -d $T --samples 40000 -O hex --output-file /dev/null
}
