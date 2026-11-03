# The IKALOGIC ScanaPLUS (emu/src/models/ikalogic-scanaplus.rs): an
# FT232H in sync FIFO mode in front of an FPGA that streams run length
# compressed pairs (a count and channel 9, then channels 1 to 8). The
# driver drops its first 64 KiB read and decodes from the second.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=ikalogic-scanaplus
EMU_OPTIONS=""
EMU_DRIVER=ikalogic-scanaplus
# The driver goes through libftdi, not usb.c; libftdi is not measured.
EMU_GCOV_SHARED="src/usb.c"
# oxrok has no ikalogic-scanaplus driver: these are oracle cases.
EMU_FULL=""

# The model's options for the cases that follow; none restores
# EMU_OPTIONS.
scanaplus_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "ikalogic-scanaplus: the emulator refused '$*'"
}

emu_cases() {
    D=ikalogic-scanaplus
    # A long capture keeps its first 16 and its last output lines, and
    # --digest the SHA-256 of all of it.
    LONG='17,$ {$!d}'
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "get samplerate" -- -d $D --get samplerate
    # LIMIT_SAMPLES is SET only: config_get answers SR_ERR_NA.
    emu_case "get limit_samples" --stderr --expect-fail -- -d $D --get limit_samples
    emu_case "set samplerate" --no-output -- -d $D --set -c samplerate=100m
    emu_case "samplerate not 100 MHz" --stderr --expect-fail -- -d $D -c samplerate=1m --samples 10
    emu_case "100 samples" -- -d $D --samples 100 -O hex
    emu_case "csv" -- -d $D --samples 40 -O csv:header=false
    emu_case "channels" -- -d $D --samples 200 -C 1,8,9 -O hex
    # Past the first decoded read: 1 M samples span two.
    emu_case "1 M samples" --digest --filter "$LONG" -- -d $D --samples 1000000 -O hex
    # LIMIT_MSEC counts 100 000 samples per ms, no clock.
    emu_case "time 1 ms" --digest --filter "$LONG" -- -d $D --time 1 -O hex
    # Both limits: the driver tests the sample limit first on each
    # decoded block, so a block that passes both ends on the sample
    # limit even when it is the larger (300 000 samples for 1 ms). The
    # time limit wins only when the block stays below the sample limit
    # (a block holds about 655 000 samples here).
    emu_case "time 1 ms, 300000 samples" --digest --filter "$LONG" -- -d $D --time 1 --samples 300000 -O hex
    emu_case "time 1 ms, 700000 samples" --digest --filter "$LONG" -- -d $D --time 1 --samples 700000 -O hex
    emu_case "samples beat time" -- -d $D --time 1 --samples 64 -O hex
    # The driver has no scan options: sigrok-cli refuses conn=.
    emu_case "by conn" --stderr --expect-fail -- -d $D:conn=$EMU_USB --samples 16 -C 9 -O hex
    emu_case "trigger" --stderr --expect-fail -- -d $D --samples 10 --triggers 1=1

    # Another device id: bit 7 set in every byte, which the driver
    # clears before sending it.
    scanaplus_options devid=ffd5aa
    emu_case "devid ffd5aa" -- -d $D --samples 16 -O hex
    scanaplus_options devid=800000
    emu_case "devid 800000" -- -d $D --samples 16 -O hex
    # The driver opens by the product string, which libftdi compares
    # exactly (the emulator's options take no spaces, so no "Single
    # RS232-HS" of a plain FT232H).
    scanaplus_options product=ScanaPLUS
    emu_case "product ScanaPLUS" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options product=
    emu_case "no product string" --stderr --expect-fail -- -d $D --samples 10

    # The FTDI shell's faults (emu/src/shells/ftdi.rs). A chip with
    # nothing yet answers its status bytes alone: reads of 0 bytes,
    # which the driver skips.
    scanaplus_options idle=3
    emu_case "idle reads" -- -d $D --samples 100 -O hex
    # The bulk IN endpoint halts: the read fails, the acquisition stops.
    scanaplus_options in-stall=0
    emu_case "first read error" --stderr -- -d $D --samples 100 -O hex
    scanaplus_options in-stall=7
    emu_case "read error after the dropped read" --stderr -- -d $D --samples 1000000 -O hex
    # The requests of the scan and of dev_open, each STALLed in turn:
    # libftdi's reset in ftdi_usb_open_desc (the scan logs it), the
    # purge, the bit mode reset, sync FIFO mode, the latency timer, the
    # EEPROM read of word 16.
    scanaplus_options stall=reset/0
    emu_case "open reset stalls" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options stall=reset/1
    emu_case "purge stalls" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options stall=bitmode/0x00ff
    emu_case "bit mode reset stalls" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options stall=bitmode/0x40ff
    emu_case "sync FIFO stalls" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options stall=latency
    emu_case "latency timer stalls" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options stall=read-eeprom
    emu_case "EEPROM read stalls" --stderr --expect-fail -- -d $D --samples 10
    scanaplus_options

    # Unplugs, counted in the host's transfers. Endpoint 0's control
    # OUTs: the scan's open sends 2, so the device is gone when
    # dev_open opens it again.
    emu_case "unplug after the scan" --unplug out=0:2 --stderr --expect-fail -- -d $D --samples 10
    # Control reads: 2 string reads by the scan's open, 2 by dev_open's,
    # then EEPROM words 16 and 17.
    emu_case "unplug before EEPROM word 17" --unplug in=0:5 --stderr --expect-fail -- -d $D --samples 10
    emu_case "unplug before write 1" --unplug in=0:6 --stderr --expect-fail -- -d $D --samples 10
    # Each write of scanaplus_init() and scanaplus_start_acquisition()
    # with its own error return: write N fails after N-1 bulk OUTs.
    # 2 to 5 the init's head, 6 and 7 the first 0x8d pulse, 120 to 122
    # the init's id, 123 its last, 124 and 125 the start's thresholds
    # and configuration, 126 to 128 the cleared id, 129 the id again.
    for n in 2 3 4 5 6 7 120 121 122 123 124 125 126 127 128 129; do
        emu_case "unplug before write $n" --unplug out=2:$((n - 1)) --stderr --expect-fail -- -d $D --samples 10
    done
    # Mid capture: the third bulk IN of the dropped read.
    emu_case "unplug mid capture" --unplug in=1:3 --stderr -- -d $D --samples 100 -O hex
}
