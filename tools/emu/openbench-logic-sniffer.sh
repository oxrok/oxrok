# An Open Bench Logic Sniffer or another SUMP board
# (emu/src/models/openbench-logic-sniffer.rs) on the CDC-ACM shell. Sourced
# by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=openbench-logic-sniffer
EMU_CDC=1
EMU_DRIVER=ols
EMU_GCOV=openbench-logic-sniffer
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
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
    emu_case "scan a short ID" --stderr --keep-header -- -d $O --scan
    emu_options id=41424344
    emu_case "scan another ID" --stderr --keep-header -- -d $O --scan
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
    emu_case "set samplerate too low" --stderr --expect-fail -- -d $O -c samplerate=9 --set
    emu_case "set samplerate too high" --stderr --expect-fail -- -d $O -c samplerate=201m --set
    emu_case "set samplerate 0" --stderr --expect-fail -- -d $O -c samplerate=0 --set
    emu_case "set limit_samples 3" --stderr --expect-fail -- -d $O -c limit_samples=3 --set
    emu_case "set limit_samples 0" --stderr --expect-fail -- -d $O -c limit_samples=0 --set
    emu_case "set captureratio 101" --stderr --expect-fail -- -d $O -c captureratio=101 --set
    emu_case "set pattern unknown" --stderr --expect-fail -- -d $O -c pattern=Bogus --set
    # config_set takes any other string for rising (api.c:329-335).
    emu_case "set clock_edge unknown" --no-output -- -d $O -c clock_edge=up --set
    emu_case "set several" --no-output -- -d $O -c rle=on:external_clock=on:clock_edge=falling --set
    # A board's maximum: 100 MHz, no demux, and one below the default.
    emu_options rate=100000000
    emu_case "max rate show" --keep-header -- -d $O --show
    emu_case "max rate refuses" --stderr --expect-fail -- -d $O -c samplerate=200m --set
    emu_case "max rate at it" -- -d $O -c samplerate=100m --get samplerate
    emu_options rate=15000000
    emu_case "max rate off the clock" -- -d $O -c samplerate=15m --get samplerate
    emu_options rate=100000
    emu_case "max rate below the default" -- -d $O --get samplerate
    emu_case "max rate below the default capture" -- -d $O --samples 8 -O hex
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
    emu_case "time 100us" --keep-header -- -d $O -c samplerate=1m --time 1ms -O hex
    for spec in 0-7 8-15 16-23 24-31 0-15 16-31 0-7,16-23 8-15,24-31 0-23 8-31 0-7,16-31 0,31 4; do
        emu_case "channels $spec" -- -d $O -C $spec --samples 8 -O hex
    done
    emu_case "channels 0-7 max" --digest --filter '$=; d' -- -d $O -C 0-7 --samples 24576 -O hex
    emu_case "channels 0-15 beyond" --stderr --expect-fail -- -d $O -C 0-15 --samples 13000 -O hex
    # The list says 24576 / 3, and three groups take four bytes each. The
    # device sends what it has, and after 100 ms of silence that is all.
    emu_case "channels 0-23 listed" --stderr --filter '/^(0|[1-9][0-9]*):/d' -- -d $O -C 0-23 --samples 8192 -O hex
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

    # RLE. slow=N holds the pins for N samples.
    emu_case "rle no runs" -- -d $O -c rle=on --samples 16 -O hex
    emu_case "rle one group" -- -d $O -c rle=on -C 8-15 --samples 1000 -O hex
    emu_case "rle two groups" -- -d $O -c rle=on -C 8-15,24-31 --samples 1000 -O hex
    emu_case "rle cut by the limit" -- -d $O -c rle=on -C 8-15 --samples 100 -O hex
    emu_options slow=100
    emu_case "rle three groups" -- -d $O -c rle=on -C 0-23 --samples 1000 -O hex
    emu_case "rle four groups" -- -d $O -c rle=on --samples 1000 -O hex
    emu_case "rle runs longer than 127" -- -d $O -c rle=on -C 0-7 --samples 1000 -O hex
    emu_options slow=300
    emu_case "rle a count in one group" -- -d $O -c rle=on -C 8-15 --samples 1000 -O hex
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
    emu_case "five stages" --stderr --expect-fail --filter '/Cannot remove non-existing event source/d' -- -d $O -c captureratio=50 --samples 16 -t 4=10101
    # A condition sigrok-cli refuses by the list of matches.
    emu_case "trigger rising" --stderr --expect-fail -- -d $O -c captureratio=50 --samples 16 -t 4=r
    emu_case "trigger falling" --stderr --expect-fail -- -d $O -c captureratio=50 --samples 16 -t 4=f
    # A channel left out: the condition is dropped, the stage stays, and
    # fires at once.
    emu_case "trigger on a disabled channel" --stderr --expect-fail -- -d $O -C 0-7 -c captureratio=50 --samples 16 -t 12=1 -O hex
    emu_case "trigger on an enabled group" -- -d $O -C 0-7 -c captureratio=50 --samples 16 -t 4=1 -O hex
    for r in 0 1 25 49 50 75 99 100; do
        emu_case "ratio $r" -- -d $O -c captureratio=$r --samples 16 -t 4=1 -O hex
    done
    emu_case "ratio 0 two stages" -- -d $O -c captureratio=0 --samples 16 -t 4=10 -O hex
    emu_case "ratio 0 three stages" -- -d $O -c captureratio=0 --samples 16 -t 4=101 -O hex
    emu_case "ratio 100 two stages" -- -d $O -c captureratio=100 --samples 16 -t 4=10 -O hex
    emu_case "ratio 100 limit 5" --expect-fail -- -d $O -c captureratio=100 --samples 5 -t 4=1 -O hex
    emu_case "ratio 50 limit 5" -- -d $O -c captureratio=50 --samples 5 -t 4=1 -O hex
    emu_case "ratio 50 limit 101" -- -d $O -c captureratio=50 --samples 101 -t 4=1 -O hex
    emu_case "ratio 50 rle" -- -d $O -c captureratio=50:rle=on -C 8-15 --samples 1000 -t 9=1 -O hex
    emu_case "ratio 50 200m" -- -d $O -c captureratio=50:samplerate=200m -C 0-15 --samples 16 -t 4=1 -O hex
    emu_case "ratio 50 beyond the memory" --stderr --expect-fail -- -d $O -c captureratio=50 --samples 7000 -t 4=1 -O hex
    emu_case "ratio 50 trigger bits" -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O bits
    emu_case "ratio 50 trigger csv" -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O csv:header=false:time=false

    # Metadata that makes the capture go wrong.
    # unitsize stays 0 without a channel count (protocol.c:304 is in
    # ols_get_metadata alone, and needs the key), and hex.c:163's loop adds
    # 0 for ever.
    emu_options meta=none
    emu_case "no metadata capture" --hangs --no-output --timeout 3 -- -d $O --samples 8 -O hex
    emu_options meta=${NAME}2100006000230bebc20000
    emu_case "no channel key capture" --hangs --no-output --timeout 3 -- -d $O --samples 8 -O hex
    # max_samples 0 makes the read count 0, sent as 0xffff (protocol.c:597-599,
    # 656), so the board sends all it has and C keeps the newest.
    emu_options memory=64 meta=${NAME}402000
    emu_case "no memory key capture" -- -d $O --samples 8 -O hex
    emu_options meta=${NAME}20000000402100006000
    emu_case "64 channels capture" -- -d $O --samples 8 -O hex
    emu_options

    # What the device does wrong.
    emu_options sends=3
    emu_case "sends 3 of 16" -- -d $O --samples 16 -O hex
    emu_case "sends 3 ratio 50" --expect-fail -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O hex
    emu_options sends=0
    emu_case "sends nothing" --hangs --no-output --timeout 2 -- -d $O --samples 16 -O hex
    emu_options
    emu_case "a trigger that never comes" --hangs --no-output --timeout 2 -- -d $O --samples 16 -t 0=1,16=1 -O hex
    emu_options chunk=1
    emu_case "one byte per transfer" -- -d $O --samples 16 -O hex
    # A stalled read loses what comes with the kernel's recovery, so only
    # the ID's is used: a capture would wait for bytes gone by timing.
    emu_options in-stall=0
    emu_case "the ID read stalled" --keep-header -- -d $O --scan
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
    emu_options sends=0
    emu_case "unplug waiting" --expect-fail --no-output --unplug model --timeout 10 --tx-filter '/^unplug$/q' -- -d $O --samples 16 -O hex
    emu_case "unplug waiting trigger" --expect-fail --unplug model --timeout 10 --tx-filter '/^unplug$/q' -- -d $O -c captureratio=50 --samples 16 -t 4=1 -O hex
    # The start's writes failing, one each: the scan writes 7, a capture
    # the stage's 3, the divider, the counts, the flags and the run.
    emu_options
    emu_case "unplug at the divider" --unplug out=2:10 --timeout 10 --stderr --expect-fail --filter '/Cannot remove non-existing event source/d' -- -d $O --samples 16 -O hex
    emu_case "unplug at the run" --unplug out=2:13 --timeout 10 --stderr --expect-fail --filter '/Cannot remove non-existing event source/d' -- -d $O --samples 16 -O hex
    emu_options in-limit=100
    emu_case "unplug mid capture" --unplug model --timeout 10 --tx-filter '/^unplug$/q' -- -d $O --samples 100 -O hex
    emu_options

    # What sigrok-cli cannot reach: a stop, and no limit.
    emu_options sends=0
    emu_case "lib stop waiting" --libsigrok --no-output --timeout 5 -- -d $O --samples 16 --stop-after-ms 300
    emu_options
    # g_try_malloc(0) is NULL, so "Sample buffer malloc failed." and the
    # source goes (protocol.c:381-385), and with it the session.
    emu_options memory=64
    emu_case "lib no limit" --libsigrok --stderr --filter 's/0x[0-9a-f]+/PTR/' -- -d $O -O hex
    # A device that stops reading: C's drain has no bound, and each copy of
    # this case costs a closing_wait.
    emu_options out-hold=0
    emu_tty_closing_wait 100
    emu_case "device stops reading" --hangs --no-output --timeout 3 --tx-filter '/^bulk_out 02 /d' -- -d $O --scan
    emu_tty_closing_wait 3000
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    O=ols:conn=$EMU_TTY
}
