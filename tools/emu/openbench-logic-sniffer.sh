# An Open Bench Logic Sniffer or another SUMP board
# (emu/src/models/openbench-logic-sniffer.rs) on the CDC-ACM shell. Sourced
# by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=openbench-logic-sniffer
EMU_CDC=1
EMU_DRIVER=ols
EMU_FULL=$EMU_DRIVER
EMU_GCOV=openbench-logic-sniffer
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix as sigrok-cli's, on its side only, and sr_strerror's
    # wording of what it adds to an error.
    L='s/^oxrok: err: oxrok::drivers::ols: /sr: openbench-logic-sniffer: /'
    A='s/^(Failed to set device option .[a-z_]+.: invalid argument): (it |invalid value for captureratio).*$/\1./
s/^(Failed to set device option .samplerate.): invalid argument: (invalid samplerate)\.?$/\1: \2./
s/^(Failed to set device option .(limit_samples|pattern).): invalid argument: .*$/\1: generic\/unspecified error./'
    # A capture that does not start: libsigrok's session and serial lines
    # have no counterpart in oxrok, and each tool ends in its own words.
    F="/^sr: (session|serial-libsp): /d; $L; \$ { /^sr: openbench-logic-sniffer: /! s/.*/start refused/; }"
    W='s/^oxrok: warn: oxrok::cli::pipeline: //'
    UNIT_BUG="unitsize comes only from a channel count in the metadata (protocol.c:304), else 0, and hex.c:163's loop adds 0 for ever: every board that answers no metadata hangs a capture; oxrok takes the size from the channels"
    MEMORY_BUG="max_samples 0 makes samplecount 0 and the read count 0 (protocol.c:597-599), sent minus 1 as 0xffff (protocol.c:656): the board sends all it has, for any limit, and the delay is as large; oxrok counts from the limit"
    DELAY_BUG="a ratio that leaves less than one unit makes delaycount 0 (protocol.c:617), sent minus 1 as 0xffff (protocol.c:656-657): the capture goes on 2^18 words past the trigger, where the marker says before it; oxrok sends one unit, and everything else as C"
    MARKER_BUG="trigger_at_smpl counts samples of a whole read (protocol.c:618), whatever the limit cut off; oxrok follows the board's words"
    BEFORE_BUG="with ratio 0 and two stages or more trigger_at_smpl is -num_stages, which only -1 passes (protocol.c:503, 526-538): num_stages samples from before raw_sample_buf go out; oxrok sends no marker, there is none in the capture"
    WRAP_BUG="a marker beyond the samples that came wraps logic.length (protocol.c:532-533), and the output runs off: SIGSEGV; oxrok sends what came with no marker"
    EMPTY_BUG="a capture with no sample sends a logic packet of length 0, and hex.c:163's length - unitsize wraps: SIGSEGV; oxrok sends no packet"
    SLOW_BUG="the scan's ols_set_samplerate(DEFAULT_SAMPLERATE) (api.c:224) fails on a board below 200 kHz, leaving 0 Hz, divider 0 and no noise filter flag; oxrok starts at the highest rate at or below the maximum, its own choice"
    DRAIN_BUG="serial_drain has no bound (serial_libsp.c:157, sp_drain), so a device that stops reading holds the scan; oxrok gives up after a second, no device"
    SAYS_SO="libsigrok has no word for a limit beyond what the counts hold, only sigrok-cli's 'Device only sent' at the end; oxrok warns ahead"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # The token block's pieces, as hex.
    NAME=014f4c5300
    # The rest of a plain board: 24576 bytes, 200 MHz, 32 channels.
    REST=2100006000230bebc200402000

    # What a scan finds: the ID, each form of the metadata.
    emu_options
    emu_case "scan" --keep-header -- -d $O --scan
    emu_case "show" --keep-header -- -d $O --show
    emu_options id=sla
    emu_case "scan SLA1" --keep-header -- -d $O --scan
    emu_options id=none
    emu_case "scan no ID" --keep-header --no-output -- -d $O --scan
    emu_options id=4142
    emu_case "scan a short ID" --stderr --filter "$L" --keep-header -- -d $O --scan
    emu_options id=41424344
    emu_case "scan another ID" --stderr --filter "$L" --keep-header -- -d $O --scan
    # What is left of the ID is taken for the metadata, as 0x58 (type 2).
    emu_options id=31534c4f58
    emu_case "scan a long ID" --keep-header -- -d $O --scan
    emu_options meta=none
    emu_case "scan no metadata" --keep-header -- -d $O --scan
    emu_case "show no metadata" --keep-header -- -d $O --show
    emu_case "get limit_samples no metadata" -- -d $O --get limit_samples
    emu_options meta=$NAME${REST}0102330000
    emu_case "scan ends at the first end" --keep-header -- -d $O --scan
    # Both orders of the versions, and the tokens that change nothing:
    # unknown ones of each type, dynamic memory, the protocol in 32 bits,
    # a key of type 3 skipped alone.
    emu_options meta=014f4c5300034100024600034200047878002200001000230bebc2002100006000240000000225000000074020410142076000
    emu_case "metadata variants" --keep-header -- -d $O --scan
    emu_case "metadata variants show" --keep-header -- -d $O --show
    emu_options meta=014f4c5300200000000800
    emu_case "metadata 32 bit channels" --keep-header -- -d $O --scan
    emu_options meta=${NAME}4010210000600000
    emu_case "metadata 16 channels" --keep-header -- -d $O --scan
    emu_case "metadata 16 channels capture" -- -d $O --samples 8 -O hex
    emu_options meta=${NAME}4004210000600000
    emu_case "metadata 4 channels capture" -- -d $O --samples 8 -O hex
    # Cut off: the rest of the block never comes.
    emu_options meta=01414243
    emu_case "metadata cut in the name" --keep-header -- -d $O --scan
    emu_options meta=${NAME}2100
    emu_case "metadata cut in a long value" --keep-header -- -d $O --scan
    emu_options meta=${NAME}40
    emu_case "metadata cut in a byte" --keep-header -- -d $O --scan
    emu_options meta=00
    emu_case "metadata only the end" --keep-header -- -d $O --scan
    # The Shrimp's defaults.
    emu_options meta=01$(emu_hex Shrimp1.0)0000
    emu_case "shrimp" --keep-header -- -d $O --show
    emu_case "shrimp capture" -- -d $O --samples 8 -O hex
    emu_options name=Shrimp1.0 channels=8 memory=4096 rate=50000000
    emu_case "shrimp keeps what it says" --keep-header -- -d $O --show
    # An FPGA version that has the flag set, which nothing reads.
    emu_options fpga=2.0
    emu_case "fpga 2.0" --keep-header -- -d $O --scan

    # probe_names.
    emu_options
    emu_case "probe names" --keep-header -- -d $O:probe_names=a,b,c --scan
    emu_case "probe names capture" -- -d $O:probe_names=a,b,c --samples 4 -O hex
    emu_options channels=4
    emu_case "probe names beyond the board" --keep-header -- -d $O:probe_names=a,b,c,d,e,f --scan
    emu_options meta=none
    emu_case "probe names no metadata" --keep-header -- -d $O:probe_names=a,b,c --scan
    emu_options

    # conn and serialcomm.
    emu_case "no such node" --keep-header --no-output -- -d ols:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d ols --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d ols --show
    emu_case "serialcomm on the native port" --keep-header -- -d $O:serialcomm=9600/8n1 --scan
    emu_options uart=115200/8n1
    emu_case "serialcomm behind a UART" --keep-header -- -d $O --scan
    emu_options uart=9600/8n1
    emu_case "serialcomm misset behind a UART" --keep-header --no-output -- -d $O --scan
    emu_case "serialcomm as the UART" --keep-header -- -d $O:serialcomm=9600/8n1 --scan
    emu_case "capture behind a UART" -- -d $O:serialcomm=9600/8n1 --samples 8 -O hex
    emu_case "trigger behind a UART" -- -d $O:serialcomm=9600/8n1 -c captureratio=50 --samples 16 -t 4=1 -O bits
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $O:serialcomm=fast --scan
    emu_options

    # Each option.
    for key in conn samplerate captureratio limit_samples external_clock clock_edge pattern rle; do
        emu_case "get $key" -- -d $O --get $key
    done
    emu_case "get swap" --stderr --expect-fail -- -d $O --get swap
    # It holds across -c.
    for setting in samplerate=1k samplerate=1m samplerate=3m samplerate=100m samplerate=150m \
        samplerate=200m samplerate=10 samplerate=11 captureratio=50 captureratio=100 \
        limit_samples=4 limit_samples=1000 external_clock=on external_clock=off \
        clock_edge=rising clock_edge=falling pattern=None pattern=External pattern=Internal rle=on rle=off; do
        key=${setting%%=*}
        emu_case "set $setting" -- -d $O -c $setting --get $key
    done
    emu_case "set swap" --no-output -- -d $O -c swap=on --set
    emu_case "set swap off" --no-output -- -d $O -c swap=off --set
    emu_case "set samplerate too low" --stderr --expect-fail --filter "$A" -- -d $O -c samplerate=9 --set
    emu_case "set samplerate too high" --stderr --expect-fail --filter "$A" -- -d $O -c samplerate=201m --set
    emu_case "set samplerate 0" --stderr --expect-fail --filter "$A" -- -d $O -c samplerate=0 --set
    emu_case "set limit_samples 3" --stderr --expect-fail --filter "$A" -- -d $O -c limit_samples=3 --set
    emu_case "set limit_samples 0" --stderr --expect-fail --filter "$A" -- -d $O -c limit_samples=0 --set
    emu_case "set captureratio 101" --stderr --expect-fail --filter "$A" -- -d $O -c captureratio=101 --set
    emu_case "set pattern unknown" --stderr --expect-fail --filter "$A" -- -d $O -c pattern=Bogus --set
    # config_set takes any other string for rising (api.c:329-335).
    emu_case "set clock_edge unknown" --no-output -- -d $O -c clock_edge=up --set
    emu_case "set several" --no-output -- -d $O -c rle=on:external_clock=on:clock_edge=falling --set
    # A board's maximum: 100 MHz, no demux, and one below the default.
    emu_options rate=100000000
    emu_case "max rate show" --keep-header -- -d $O --show
    emu_case "max rate refuses" --stderr --expect-fail --filter "$A" -- -d $O -c samplerate=200m --set
    emu_case "max rate at it" -- -d $O -c samplerate=100m --get samplerate
    emu_options rate=15000000
    emu_case "max rate off the clock" -- -d $O -c samplerate=15m --get samplerate
    emu_options rate=150000
    emu_case "max rate 150 kHz" --oracle-differs "$SLOW_BUG" -- -d $O --get samplerate
    emu_case "max rate 150 kHz capture" --oracle-differs "$SLOW_BUG" -- -d $O --samples 8 -O hex
    emu_options rate=100000
    emu_case "max rate below the default" --oracle-differs "$SLOW_BUG" -- -d $O --get samplerate
    emu_case "max rate below the default capture" --oracle-differs "$SLOW_BUG" -- -d $O --samples 8 -O hex
    emu_options

    # Captures. What differs between them is on the wire.
    for n in 4 5 6 7 8 9 16 100 1000; do
        emu_case "samples $n" -- -d $O --samples $n -O hex
    done
    emu_case "samples 3" --stderr --expect-fail -- -d $O --samples 3
    emu_case "samples 0" --stderr --expect-fail -- -d $O --samples 0
    emu_case "samples 16 bits" -- -d $O --samples 16 -O bits
    emu_case "samples 16 csv" -- -d $O --samples 16 -O csv:header=false:time=false
    emu_case "samples 6144" --digest --filter '$=; d' -- -d $O --samples 6144 -O hex
    # More than the memory: sigrok-cli's g_critical is fatal.
    emu_case "samples 7000" --stderr --expect-fail -- -d $O --samples 7000 -O hex
    emu_case "time 1ms" -- -d $O --time 1ms -O hex
    # Beyond the memory: no g_critical on this path, and what the board has.
    emu_case "time beyond the memory" --stderr --filter "/^(0|[1-9][0-9]*):/d
$W
/^oxrok: warn: oxrok::drivers::ols::protocol: The board returns /d" -- -d $O --time 100ms -O hex
    emu_case "time 100us" --keep-header -- -d $O -c samplerate=1m --time 1ms -O hex
    for spec in 0-7 8-15 16-23 24-31 0-15 16-31 0-7,16-23 8-15,24-31 0-23 8-31 0-7,16-31 0,31 4; do
        emu_case "channels $spec" -- -d $O -C $spec --samples 8 -O hex
    done
    emu_case "channels 0-7 max" --digest --filter '$=; d' -- -d $O -C 0-7 --samples 24576 -O hex
    emu_case "channels 0-15 beyond" --stderr --expect-fail -- -d $O -C 0-15 --samples 13000 -O hex
    # The list says 24576 / 3, and three groups take four bytes each. The
    # device sends what it has, and after 100 ms of silence that is all.
    emu_case "channels 0-23 listed" --stderr --filter "/^(0|[1-9][0-9]*):/d
$W" -- -d $O -C 0-23 --samples 8192 -O hex
    for rate in 10 1k 200k 1m 25m 100m; do
        emu_case "rate $rate" -- -d $O -c samplerate=$rate --samples 8 -O hex
    done
    # Demux: every C sample is a word of two.
    emu_case "rate 200m" -- -d $O -c samplerate=200m --samples 8 -O hex
    emu_case "rate 200m 16 channels" -- -d $O -c samplerate=200m -C 0-15 --samples 8 -O hex
    emu_case "rate 150m" -- -d $O -c samplerate=150m --samples 8 -O hex
    for setting in external_clock=on external_clock=on:clock_edge=falling pattern=Internal pattern=External swap=on; do
        emu_case "capture $setting" -- -d $O -c $setting --samples 8 -O hex
    done
    # Back off again before the capture.
    emu_case "capture pattern back to none" -- -d $O -c pattern=Internal:pattern=None --samples 8 -O hex

    # Memory: at 256 KiB still 0x81; above it 0x84 and 0x83.
    emu_options memory=262144
    emu_case "memory 256K" -- -d $O --samples 16 -O hex
    emu_options memory=262148
    emu_case "memory over 256K" -- -d $O --samples 16 -O hex
    emu_case "memory over 256K trigger" -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O hex
    emu_options meta=${NAME}4020210000600000
    # No maximum rate in the block: the default table's.
    emu_case "no rate key" --keep-header -- -d $O --show
    emu_options

    # RLE. slow=N holds the pins for N samples. What a limit leaves is still
    # coming while the resets go out, so a capture stays within one bulk IN
    # of the kernel's 128 bytes, and the .tx has one order.
    emu_case "rle no runs" -- -d $O -c rle=on --samples 16 -O hex
    emu_case "rle one group" -- -d $O -c rle=on -C 8-15 --samples 128 -O hex
    emu_case "rle two groups" -- -d $O -c rle=on -C 8-15,24-31 --samples 64 -O hex
    emu_case "rle cut by the limit" -- -d $O -c rle=on -C 8-15 --samples 100 -O hex
    emu_options slow=8
    emu_case "rle three groups" -- -d $O -c rle=on -C 0-23 --samples 32 -O hex
    emu_case "rle four groups" -- -d $O -c rle=on --samples 32 -O hex
    emu_options slow=300
    emu_case "rle runs longer than 127" -- -d $O -c rle=on -C 0-7 --samples 128 -O hex
    emu_options memory=64 slow=8
    emu_case "rle at the memory" -- -d $O -c rle=on -C 0-7 --samples 64 -O hex
    emu_case "rle beyond the memory" --stderr --expect-fail -- -d $O -c rle=on -C 0-7 --samples 4096 -O hex
    emu_options

    # Triggers: the stages, the marker, the ratio.
    for spec in 4=1 4=0 0=1,1=1 0=1,16=0 1=1,8=1,17=0; do
        emu_case "trigger $spec" -- -d $O -c captureratio=50 --samples 16 -t $spec -O hex
    done
    # Stages: more than one level.
    emu_case "two stages" -- -d $O -c captureratio=50 --samples 16 -t 4=10 -O hex
    emu_case "three stages" -- -d $O -c captureratio=50 --samples 16 -t 4=101 -O hex
    emu_case "four stages" -- -d $O -c captureratio=50 --samples 16 -t 4=1010 -O hex
    # The session's stop removes a source there is none of, and logs where.
    emu_case "five stages" --stderr --expect-fail --filter "$F" -- -d $O -c captureratio=50 --samples 16 -t 4=10101
    # A condition sigrok-cli refuses by the list of matches.
    emu_case "trigger rising" --stderr --expect-fail -- -d $O -c captureratio=50 --samples 16 -t 4=r
    emu_case "trigger falling" --stderr --expect-fail -- -d $O -c captureratio=50 --samples 16 -t 4=f
    # A channel left out: the condition is dropped, the stage stays, and
    # fires at once.
    emu_case "trigger on a disabled channel" --stderr --expect-fail -- -d $O -C 0-7 -c captureratio=50 --samples 16 -t 12=1 -O hex
    emu_case "trigger on an enabled group" -- -d $O -C 0-7 -c captureratio=50 --samples 16 -t 4=1 -O hex
    for r in 0 1 25 49 50 75; do
        emu_case "ratio $r" -- -d $O -c captureratio=$r --samples 16 -t 4=1 -O hex
    done
    for r in 99 100; do
        emu_case "ratio $r" --oracle-differs "$DELAY_BUG" -- -d $O -c captureratio=$r --samples 16 -t 4=1 -O hex
    done
    emu_case "ratio 0 two stages" --oracle-differs "$BEFORE_BUG" -- -d $O -c captureratio=0 --samples 16 -t 4=10 -O hex
    emu_case "ratio 0 three stages" --oracle-differs "$BEFORE_BUG" -- -d $O -c captureratio=0 --samples 16 -t 4=101 -O hex
    emu_case "ratio 100 two stages" --oracle-differs "$DELAY_BUG" -- -d $O -c captureratio=100 --samples 16 -t 4=10 -O hex
    emu_case "ratio 100 limit 5" --oracle-bug "$WRAP_BUG" -- -d $O -c captureratio=100 --samples 5 -t 4=1 -O hex
    # The output shows no marker in so few samples, where the tools differ
    # by 3: a unit test has oxrok's.
    emu_case "ratio 50 limit 5" -- -d $O -c captureratio=50 --samples 5 -t 4=1 -O hex
    emu_case "ratio 50 limit 101" --oracle-differs "$MARKER_BUG" -- -d $O -c captureratio=50 --samples 101 -t 4=1 -O bits
    emu_case "ratio 50 rle" -- -d $O -c captureratio=50:rle=on -C 8-15 --samples 128 -t 9=1 -O bits
    emu_options slow=8
    emu_case "ratio 50 rle marker" -- -d $O -c captureratio=50:rle=on -C 0-7 --samples 64 -t 4=1 -O bits
    emu_options
    # So few samples that the ratio leaves C a delay of 0, 0xffff on the wire.
    emu_case "ratio 50 limit 4" --oracle-differs "$DELAY_BUG" -- -d $O -c captureratio=50 --samples 4 -t 4=1 -O bits
    emu_case "ratio 100 limit 8" --oracle-differs "$DELAY_BUG" -- -d $O -c captureratio=100 --samples 8 -t 4=1 -O bits
    emu_case "ratio 1 limit 13 two stages" -- -d $O -c captureratio=1 --samples 13 -t 4=10 -O bits
    emu_case "ratio 50 200m" -- -d $O -c captureratio=50:samplerate=200m -C 0-15 --samples 16 -t 4=1 -O hex
    emu_case "ratio 50 beyond the memory" --stderr --expect-fail -- -d $O -c captureratio=50 --samples 7000 -t 4=1 -O hex
    emu_case "ratio 50 trigger bits" -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O bits
    emu_case "ratio 50 trigger csv" -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O csv:header=false:time=false

    # Metadata that makes the capture go wrong.
    # unitsize stays 0 without a channel count (protocol.c:304 is in
    # ols_get_metadata alone, and needs the key), and hex.c:163's loop adds
    # 0 for ever.
    emu_options meta=none
    emu_case "no metadata capture" --oracle-bug "$UNIT_BUG" --oracle-hangs --timeout 5 -- -d $O --samples 8 -O hex
    emu_case "no metadata trigger" --oracle-bug "$UNIT_BUG" --oracle-hangs --timeout 5 -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O bits
    emu_options meta=${NAME}2100006000230bebc20000
    emu_case "no channel key capture" --oracle-bug "$UNIT_BUG" --oracle-hangs --timeout 5 -- -d $O --samples 8 -O hex
    # max_samples 0 makes the read count 0, sent as 0xffff (protocol.c:597-599,
    # 656), so the board sends all it has and C keeps the newest.
    emu_options memory=64 meta=${NAME}402000
    emu_case "no memory key capture" --oracle-differs "$MEMORY_BUG" -- -d $O --samples 8 -O hex
    # Past what the 16 bit counts hold, both tools send 0xffff, and the board
    # gives its 2^18 words. A memory of 1 MiB holds them. oxrok says so.
    emu_options memory=1048576 meta=${NAME}402000
    emu_case "no memory key 262144" --digest --filter '$=; d' --timeout 120 -- -d $O --samples 262144 -O hex
    emu_case "no memory key 262148" --digest --filter '$=; d' --timeout 120 -- -d $O --samples 262148 -O hex
    emu_case "no memory key 300000" --digest --filter '$=; d' --timeout 120 -- -d $O --samples 300000 -O hex
    emu_case "no memory key 300000 says so" --oracle-differs "$SAYS_SO" --stderr --filter "/^(0|[1-9][0-9]*):/d
$W" --timeout 120 -- -d $O --samples 300000 -O hex
    # A time is a limit like any other: 2 s at 200 kHz is 400000.
    emu_case "no memory key time" --digest --filter '$=; d' --timeout 120 -- -d $O --time 2s -O hex
    emu_case "no memory key trigger" --oracle-differs "$MEMORY_BUG" --digest --filter '$=; d' --timeout 120 -- -d $O -c captureratio=50 --samples 300000 -t 4=1 -O hex
    emu_options meta=${NAME}20000000402100006000
    emu_case "64 channels capture" -- -d $O --samples 8 -O hex
    emu_options

    # What the device does wrong.
    emu_options sends=3
    emu_case "sends 3 of 16" --oxrok-fails "the board went silent after 3 of 16 samples" -- -d $O --samples 16 -O hex
    emu_case "sends 3 ratio 50" --oxrok-fails "the board went silent after 3 of 16 samples" --oracle-bug "$WRAP_BUG" -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O hex
    emu_options sends=0
    # Nothing ends these but a stop, under either tool: a capture that has
    # not begun has no silence to end on. What oxrok does with the stop.
    emu_case "lib sends nothing" --libsigrok --hangs --no-output --timeout 2 -- -d $O --samples 16 -O hex
    emu_case "interrupt waiting" --oracle-bug "$SIGINT_BUG" --interrupt model --no-output --timeout 15 --tx-filter '/^interrupt$/q' -- -d $O --samples 16 -O hex
    emu_options
    emu_case "lib a trigger that never comes" --libsigrok --hangs --no-output --timeout 2 -- -d $O --samples 16 -t 0=1,16=1 -O hex
    emu_case "interrupt a trigger that never comes" --oracle-bug "$SIGINT_BUG" --interrupt model --no-output --timeout 15 --tx-filter '/^interrupt$/q' -- -d $O --samples 16 -t 0=1,16=1 -O hex
    # A stop mid capture drops what came, as abort_acquisition does, with
    # 100 ms of silence racing it: a unit test has that.
    emu_options chunk=1
    emu_case "one byte per transfer" -- -d $O --samples 16 -O hex
    # No in-stall=: the kernel's recovery loses what comes with it or not, by
    # timing, and nothing here asks again.
    emu_options stall=line
    emu_case "line coding stalled" --keep-header -- -d $O --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $O --scan
    emu_options

    # Unplugs. The scan writes the resets, then the ID; a capture sends its
    # commands, then waits.
    emu_options id=none meta=none
    emu_case "unplug in the scan" --unplug out=2:2 --timeout 10 --keep-header --no-output -- -d $O --scan
    # The hangup ends the capture with no sample: an empty logic packet, and
    # the output's loop runs off (hex.c:163).
    # oxrok fails it either way, by the pace: in the start's last drain (I/O
    # error, no capture) or in the capture (Broken pipe). The .out and .tx are alike.
    emu_options sends=0
    emu_case "unplug waiting" --oxrok-fails "the port gone" --oracle-bug "$EMPTY_BUG" --no-output --unplug model --timeout 10 --tx-filter '/^unplug$/q' -- -d $O --samples 16 -O hex
    emu_case "unplug waiting trigger" --oxrok-fails "the port gone" --oracle-bug "$EMPTY_BUG" --no-output --unplug model --timeout 10 --tx-filter '/^unplug$/q' -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O hex
    # A write of the start failing: the scan writes 7, a capture the stage's
    # 3 first. Later than this the unplug and the run race, and C's hangup
    # has it end by a crash or an error as the pace has it.
    emu_options
    emu_case "unplug at the divider" --unplug out=2:10 --timeout 10 --stderr --expect-fail --filter "$F" -- -d $O --samples 16 -O hex
    # No unplug mid capture: bytes just delivered are lost to the hangup or
    # not, as the pace has it, and with none C crashes. A unit test has it.
    emu_options

    # What sigrok-cli cannot reach: a stop, and no limit.
    emu_options sends=0
    emu_case "lib stop waiting" --libsigrok --no-output --timeout 5 -- -d $O --samples 16 --stop-after-ms 300
    emu_options
    # g_try_malloc(0) is NULL, so "Sample buffer malloc failed." and the
    # source goes (protocol.c:381-385), and with it the session.
    emu_options memory=64
    emu_case "lib no limit" --libsigrok --stderr --filter 's/0x[0-9a-f]+/PTR/' -- -d $O -O hex
    # A device that stops reading: this driver drains each command, so it
    # stops at the first, a path of its own. It costs a closing_wait.
    emu_options out-hold=0
    emu_tty_closing_wait 100
    emu_case "device stops reading" --oracle-bug "$DRAIN_BUG" --oracle-hangs --no-output --keep-header --timeout 9 --tx-filter '/^bulk_out 02 /d' -- -d $O --scan
    emu_tty_closing_wait 3000
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    O=ols:conn=$EMU_TTY
}
