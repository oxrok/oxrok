# The Pipistrello LX45 with the Open Logic Sniffer bitstream
# (emu/src/models/pipistrello-ols.rs): an FT2232H whose port B carries
# SUMP commands to the FPGA and the samples back, newest first. The
# pins carry the count of samples since the run (channels 0-15) and its
# complement (16-31). Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model".
EMU_MODEL=pipistrello-ols
EMU_DRIVER=p-ols
EMU_GCOV=pipistrello-ols
# The driver goes through libftdi alone: no libsigrok transport file
# (usb.c, serial.c) is involved, so EMU_GCOV_SHARED stays empty.
EMU_GCOV_SHARED=""
# oxrok has no p-ols driver yet: these are oracle cases.
EMU_FULL=""

# The model's options for the cases that follow (see ftdi-la.sh).
pols_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "pipistrello-ols: the emulator refused '$*'"
}

emu_cases() {
    D=p-ols
    # A long output keeps its first 16 and its last lines; --digest
    # still compares every byte.
    LONG='17,$ {$!d}'
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "get samplerate" -- -d $D --get samplerate
    emu_case "get captureratio" -- -d $D --get captureratio
    emu_case "get limit_samples" -- -d $D --get limit_samples
    emu_case "get pattern" -- -d $D --get pattern
    emu_case "get rle" -- -d $D --get rle
    emu_case "get external_clock" -- -d $D --get external_clock
    emu_case "set samplerate" --no-output -- -d $D --set -c samplerate=2m
    # sigrok-cli sets the -c options before the --get.
    emu_case "get pattern external" -- -d $D -c pattern=External --get pattern
    emu_case "get pattern internal" -- -d $D -c pattern=Internal --get pattern

    # Captures, no trigger: the forced stage 0 starts at the first word.
    emu_case "100 samples" -- -d $D --samples 100 -O hex
    emu_case "1 MHz" -- -d $D -c samplerate=1m --samples 1000 -O hex
    # 100 MHz / 33: 3.03 MHz.
    emu_case "3 MHz" -- -d $D -c samplerate=3m --samples 64 -O hex
    # Slow rates take polls: each empty read is a 16 ms latency period.
    emu_case "10 kHz" -- -d $D -c samplerate=10k --samples 500 -O hex
    emu_case "10 Hz" -- -d $D -c samplerate=10 --samples 4 -O bits
    emu_case "20000 samples" --digest --filter "$LONG" -- -d $D --samples 20000 -O hex
    emu_case "time" --digest --filter "$LONG" -- -d $D -c samplerate=1m --time 5 -O hex
    # Disabled channel groups are not sent; the driver expands them.
    emu_case "one group" -- -d $D -C 8-15 --samples 300 -O hex
    emu_case "two groups" -- -d $D -C 0,1,30,31 --samples 64 -O bits
    emu_case "three groups" -- -d $D -C 0-7,16-31 --samples 64 -O hex
    emu_case "channel 31" -- -d $D -C 31 --samples 16 -O bits

    # Triggers: channel k first rises at sample 2^k. The driver adds a
    # stage that starts the capture one sample after the last of its own.
    emu_case "trigger rising" -- -d $D -C 0-7 --samples 64 -c captureratio=50 --triggers 4=r -O bits
    emu_case "trigger falling" -- -d $D -C 0-7 --samples 64 -c captureratio=25 --triggers 5=f -O bits
    emu_case "trigger level" -- -d $D -C 0-7 --samples 64 -c captureratio=50 --triggers 3=1,0=0 -O bits
    emu_case "trigger upper channel" -- -d $D -C 16-23 --samples 64 -c captureratio=50 --triggers 20=0 -O bits
    # No pre-trigger samples: no trigger marker either (trigger_at -1).
    emu_case "trigger ratio 0" -- -d $D -C 0-7 --samples 32 --triggers 4=r -O bits
    # Late: sample 1024 at 10 kHz, seven polls.
    emu_case "trigger after polls" -- -d $D -C 8-15 -c samplerate=10k:captureratio=50 --samples 32 --triggers 10=r -O bits
    # Pre-trigger samples beyond the run: memory never written reads 0.
    emu_case "trigger early" -- -d $D -C 0-7 --samples 64 -c captureratio=90 --triggers 1=r -O bits
    # Stages: one per character of a match.
    emu_case "two stages" -- -d $D -C 0-7 --samples 64 -c captureratio=50 --triggers 3=10 -O bits
    emu_case "three stages" -- -d $D -C 0-7 --samples 64 -c captureratio=50 --triggers 3=r1f -O bits
    emu_case "five stages" --stderr --expect-fail -- -d $D --samples 64 --triggers 3=10101

    # RLE: a run is a word and a count word (top bit set), counts of
    # 7 bits for one group; the top channel of the enabled groups is lost.
    emu_case "rle one group" -- -d $D -c rle=on -C 8-15 --samples 1000 -O hex
    emu_case "rle all channels" -- -d $D -c rle=on --samples 64 -O hex
    emu_case "rle two groups" -- -d $D -c rle=on -C 8-23 --samples 300 -O hex
    # A limit that is not a multiple of 4: with RLE the excess comes off
    # a run's count, which is large enough here.
    emu_case "rle 1001 samples" -- -d $D -c rle=on -C 8-15 --samples 1001 -O hex
    emu_case "rle trigger" -- -d $D -c rle=on:captureratio=50 -C 8-15 --samples 2000 --triggers 10=r -O hex

    # Demux: 200 MHz, two samples of channels 0-15 per word; the driver
    # replicates the enabled groups into the upper half.
    emu_case "200 MHz" -- -d $D -c samplerate=200m -C 0-15 --samples 96 -O hex
    emu_case "150 MHz" -- -d $D -c samplerate=150m -C 0-7 --samples 64 -O hex
    emu_case "200 MHz rle" -- -d $D -c samplerate=200m:rle=on -C 8-15 --samples 1000 -O hex
    emu_case "200 MHz rle two groups" -- -d $D -c samplerate=200m:rle=on -C 0-15 --samples 64 -O hex
    emu_case "200 MHz trigger" -- -d $D -c samplerate=200m:captureratio=50 -C 0-7 --samples 64 --triggers 5=r -O bits

    # The flags the model accepts and does not emulate: the same samples.
    emu_case "pattern internal" -- -d $D -c pattern=Internal -C 0-7 --samples 16 -O hex
    emu_case "pattern external" -- -d $D -c pattern=External -C 0-7 --samples 16 -O hex
    emu_case "pattern none" -- -d $D -c pattern=None -C 0-7 --samples 16 -O hex
    emu_case "swap" -- -d $D -c swap=on -C 0-7 --samples 16 -O hex
    emu_case "flags off" -- -d $D -c external_clock=off:swap=off:rle=off -C 0-7 --samples 16 -O hex
    # The external clock: 1 MHz on the clock input, the divider ignored.
    emu_case "external clock" -- -d $D -c external_clock=on -C 0-7 --samples 64 -O hex

    # The driver's refusals.
    emu_case "samplerate too low" --stderr --expect-fail -- -d $D -c samplerate=5 --samples 10
    emu_case "samplerate too high" --stderr --expect-fail -- -d $D -c samplerate=300m --samples 10
    emu_case "too few samples" --stderr --expect-fail -- -d $D --samples 2
    emu_case "pattern unknown" --stderr --expect-fail -- -d $D -c pattern=Other --samples 10
    emu_case "get conn" --stderr --expect-fail -- -d $D --get conn

    # More samples than the read count asks for in one buffer: the
    # driver takes the excess off rle_count, 0 without RLE, which wraps
    # to UINT_MAX, and its copy loop runs off the buffer (protocol.c
    # 557-563, 597-600). Any limit that is not a multiple of 4 (8 in
    # demux) does it.
    emu_case "101 samples" --stderr --expect-fail -- -d $D --samples 101 -O hex
    emu_case "200 MHz 100 samples" --stderr --expect-fail -- -d $D -c samplerate=200m -C 0-15 --samples 100 -O hex

    # A smaller memory: sigrok-cli refuses a limit beyond the listed
    # maximum; with RLE there is none, and the read count stops at the
    # memory.
    pols_options memory=4096
    emu_case "memory limit" --stderr --expect-fail -- -d $D --samples 2000 -O hex
    emu_case "memory limit one group" --stderr --expect-fail -- -d $D -C 0-7 --samples 5000 -O hex
    emu_case "memory limit rle" -- -d $D -c rle=on -C 8-15 --samples 5000 -O hex
    # A board whose metadata says 100 MHz at most: no demux.
    pols_options max-rate=100000000
    emu_case "max rate" --stderr --expect-fail -- -d $D -c samplerate=200m -C 0-15 --samples 10
    emu_case "max rate 100 MHz" -- -d $D -c samplerate=100m -C 0-7 --samples 16 -O hex
    # Below the driver's default 100 MHz: the scan's samplerate setting
    # fails, the rate reads 0 and a capture runs with the divider 0.
    pols_options max-rate=50000000
    emu_case "max rate 50 MHz" -- -d $D --get samplerate
    emu_case "max rate 50 MHz capture" -- -d $D -C 0-7 --samples 16 -O hex
    pols_options channels=16
    emu_case "16 channels" --keep-header -- -d $D --scan
    emu_case "16 channels capture" -- -d $D --samples 32 -O hex

    # Metadata the driver parses otherwise: ancillary and FPGA versions
    # in both orders (0x03, 0x02), unknown string, 32 and 8 bit tokens
    # (0x04, 0x25, 0x42), dynamic memory (0x22), the protocol version in
    # 32 and 8 bits (0x24, 0x41), the 8 bit channel count (0x40, 16), a
    # key of type 3 (skipped alone), no memory size (no sample limit).
    pols_options metadata=014f4c5300034100024600034200047878002200001000230bebc200240000000225000000074010410142076000
    emu_case "metadata variants" --keep-header -- -d $D --scan
    emu_case "metadata variants show" --keep-header -- -d $D --show
    # A block beyond the driver's 64 byte read: the name is cut at 63
    # bytes, the channel count never read.
    pols_options metadata=01414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414100200000002000
    emu_case "metadata long" --keep-header -- -d $D --scan
    # The FPGA answers no metadata: the driver gives up.
    pols_options metadata=none
    emu_case "metadata none" --stderr --expect-fail -- -d $D --samples 10
    # The ID: the OLS's own is accepted, no answer or another is not.
    pols_options id=ols
    emu_case "id ols" --keep-header -- -d $D --scan
    pols_options id=none
    emu_case "id none" --stderr --expect-fail -- -d $D --samples 10
    pols_options id=31534c41
    emu_case "id other" --stderr --expect-fail -- -d $D --samples 10
    # Another board on the same chip: libftdi matches the product string.
    pols_options product=Papilio
    emu_case "other product" --stderr --expect-fail -- -d $D --samples 10

    # The FTDI shell's faults (emu/src/shells/ftdi.rs).
    # The requests of p_ols_open, each STALLed: libftdi's own reset in
    # ftdi_usb_open, the purge, the bit mode reset, the latency timer.
    pols_options stall=reset/0
    emu_case "open reset stalls" --stderr -- -d $D --scan
    pols_options stall=reset/1
    emu_case "purge stalls" --stderr -- -d $D --scan
    pols_options stall=bitmode
    emu_case "bit mode stalls" --stderr -- -d $D --scan
    pols_options stall=latency
    emu_case "latency stalls" --stderr -- -d $D --scan
    # Port B's bulk IN halts: the ID read, the metadata read, the
    # capture's first read.
    pols_options in-stall=0
    emu_case "id read error" --stderr -- -d $D --scan
    pols_options in-stall=1
    emu_case "metadata read error" --stderr -- -d $D --scan
    pols_options in-stall=3
    emu_case "capture read error" --stderr -- -d $D --samples 100 -O hex
    # Status bytes alone after the bit mode reset: the ID read gets 0 bytes.
    pols_options idle=1
    emu_case "id read empty" --stderr --expect-fail -- -d $D --samples 10
    pols_options
    # Unplugged after so many writes (the scan writes 7): the scan's
    # reset, then each write of the acquisition start in turn, without a
    # trigger (stage 0 at 8-11, divider 12, read count 13, delay count
    # 14, flags 15, run 16) and with one (stages 0-1 at 8-15, the
    # disabled stage 2 at 16).
    U='/LIBUSB_ERROR_ACCESS/d'
    emu_case "unplug at the scan" --unplug out=4:1 --stderr --expect-fail --filter "$U" -- -d $D --samples 10
    emu_case "unplug at stage 0" --unplug out=4:8 --stderr --expect-fail --filter "$U" -- -d $D --samples 16
    emu_case "unplug at the divider" --unplug out=4:11 --stderr --expect-fail --filter "$U" -- -d $D --samples 16
    emu_case "unplug at the read count" --unplug out=4:12 --stderr --expect-fail --filter "$U" -- -d $D --samples 16
    emu_case "unplug at the delay count" --unplug out=4:13 --stderr --expect-fail --filter "$U" -- -d $D --samples 16
    emu_case "unplug at the flags" --unplug out=4:14 --stderr --expect-fail --filter "$U" -- -d $D --samples 16
    emu_case "unplug at the run" --unplug out=4:15 --stderr --expect-fail --filter "$U" -- -d $D --samples 16
    emu_case "unplug at a trigger stage" --unplug out=4:9 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    emu_case "unplug at a disabled stage" --unplug out=4:16 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    emu_case "unplug at a stage edge" --unplug out=4:10 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    emu_case "unplug at a disabled mask" --unplug out=4:15 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    emu_case "unplug at a disabled config" --unplug out=4:17 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    emu_case "unplug at a disabled edge" --unplug out=4:18 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    # After the open's last control request (each open sends 6): the
    # start's first write, stage 0's mask, fails.
    emu_case "unplug after the open" --unplug out=0:12 --stderr --expect-fail --filter "$U" -- -d $D --samples 16 --triggers 0=1
    # Unplugged after the capture's second transfer: the next read and
    # the resets of the stop fail.
    emu_case "unplug mid capture" --unplug in=3:5 --stderr -- -d $D --samples 20000 -O hex
}
