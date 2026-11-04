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
# oxrok has the driver (src/drivers/ikalogic_scanaplus): every case runs
# both tools. sigrok-cli runs first in each, and libftdi detaches the
# kernel's ftdi_sio from port A without attaching it again, so oxrok
# finds the port free; after an --unplug case's replug the harness unbinds it
# before oxrok's run, as sigrok-cli's would have left it. oxrok gives the
# port back when it closes (the scan's close too): with ftdi_sio bound,
# it binds again and probes port A, which sigrok-cli never causes.
EMU_FULL=ikalogic-scanaplus

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
    # The two tools prefix their log lines differently ("sr: DRIVER: "
    # and "oxrok: LEVEL: MODULE: "): the messages are compared without.
    N='s/^sr: [a-z0-9-]+: //; s/^oxrok: [a-z]+: [a-z0-9_:]+: //'
    # A failed open: sigrok-cli's own line, oxrok's adds the reason.
    O="$N; s/^(Failed to open device): .*/\1./"
    # A start that fails: libsigrok's session adds two lines ("Could not
    # start ikalogic-scanaplus device (null) acquisition." and "Cannot
    # remove non-existing event source", as it stops the device that
    # failed, session.c 831-835), then sigrok-cli's "Failed to start
    # session."; oxrok's session hands back the driver's error. Both
    # last lines are reduced to "start refused".
    S="/^sr: session: /d; $N; s/^(Failed to start session\\.|device error: libftdi error .*)\$/start refused/"
    # The C driver's results with a stale or short read (protocol.c
    # 345-352) and with both limits (protocol.c 358-367).
    DROP="protocol.c 345-350 drops the first read as if it held 64 KiB (compressed_bytes_ignored += COMPRESSED_BUF_SIZE)"
    ODD="protocol.c 352 decodes a read of odd length in pairs, with the stale byte past its end"
    BOTH="protocol.c 358-367 tests the sample limit before the time limit, so a read past both ends on the larger"
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "get samplerate" -- -d $D --get samplerate
    # LIMIT_SAMPLES is SET only: config_get answers SR_ERR_NA.
    emu_case "get limit_samples" --stderr --expect-fail -- -d $D --get limit_samples
    # Not listed at all: sigrok-cli's maybe_config_get() (main.c:81-84)
    # answers the same without asking the driver.
    emu_case "get limit_frames" --stderr --expect-fail -- -d $D --get limit_frames
    emu_case "set samplerate" --no-output -- -d $D --set -c samplerate=100m
    emu_case "samplerate not 100 MHz" --stderr --expect-fail --filter "$N; s/^(Failed to set device option .samplerate.: invalid argument).*/\\1./" -- -d $D -c samplerate=1m --samples 10
    emu_case "100 samples" --record -- -d $D --samples 100 -O hex
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
    emu_case "time 1 ms, 300000 samples" --oracle-differs "$BOTH" --digest --filter "$LONG" -- -d $D --time 1 --samples 300000 -O hex
    emu_case "time 1 ms, 700000 samples" --digest --filter "$LONG" -- -d $D --time 1 --samples 700000 -O hex
    emu_case "samples beat time" -- -d $D --time 1 --samples 64 -O hex
    # The driver has no scan options: sigrok-cli's scan finds nothing
    # (hwdriver.c 584-585, silently), oxrok names the option it refuses.
    emu_case "by conn" --stderr --expect-fail --filter "s/^Unknown scan option 'conn'\\.\$/No devices found./" -- -d $D:conn=$EMU_USB --samples 16 -C 9 -O hex
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
    emu_case "no product string" --stderr --expect-fail --filter "$N" -- -d $D --samples 10

    # The FTDI shell's faults (emu/src/shells/ftdi.rs). A chip with
    # nothing yet answers its status bytes alone: reads of 0 bytes,
    # which the driver skips.
    scanaplus_options idle=3
    emu_case "idle reads" -- -d $D --samples 100 -O hex
    # The bulk IN endpoint halts: the read fails, the acquisition stops.
    scanaplus_options in-stall=0
    emu_case "first read error" --stderr --filter "$N" -- -d $D --samples 100 -O hex
    scanaplus_options in-stall=7
    emu_case "read error after the dropped read" --stderr --filter "$N" -- -d $D --samples 1000000 -O hex
    # Latency flushes (the model's short=N:M): transfer N of the stream
    # carries M bytes, 0 the status bytes alone, which ends libftdi's
    # read. A first read of 100 bytes is dropped as if it were 64 KiB:
    # the samples start at stream byte 100, pair 50, not 65536 as in
    # "100 samples".
    scanaplus_options short=1:100,2:0
    emu_case "short first read" --oracle-differs "$DROP" -- -d $D --samples 100 -O hex
    # The first read ends after 4 full transfers (65280 bytes, dropped),
    # the second holds 3 bytes: the driver decodes 2 pairs, the second
    # with byte 3 of the buffer, left over from the first read, and
    # from the third read on every pair is split across its bytes.
    scanaplus_options short=5:0,6:3,7:0
    emu_case "odd read" --oracle-differs "$ODD" -- -d $D --samples 300 -O hex
    # oxrok drops the first 64 KiB of the stream counted in bytes (all
    # of the first two reads and 253 bytes of the third) and keeps the
    # odd byte at the end of a read for the next one: its samples here
    # and in "short first read" are those of "1 M samples" and "100
    # samples", the same digest. A capture across three reads carries
    # the odd byte over twice.
    emu_case "odd read, 1 M samples" --oracle-differs "$ODD" --digest --filter "$LONG" -- -d $D --samples 1000000 -O hex
    # The requests of the scan and of dev_open, each STALLed in turn:
    # libftdi's reset in ftdi_usb_open_desc (the scan logs it), the
    # purge, the bit mode reset, sync FIFO mode, the latency timer, the
    # EEPROM read of word 16.
    scanaplus_options stall=reset/0
    emu_case "open reset stalls" --stderr --expect-fail --filter "$O" -- -d $D --samples 10
    scanaplus_options stall=reset/1
    emu_case "purge stalls" --stderr --expect-fail --filter "$O" -- -d $D --samples 10
    scanaplus_options stall=bitmode/0x00ff
    emu_case "bit mode reset stalls" --stderr --expect-fail --filter "$O" -- -d $D --samples 10
    scanaplus_options stall=bitmode/0x40ff
    emu_case "sync FIFO stalls" --stderr --expect-fail --filter "$O" -- -d $D --samples 10
    scanaplus_options stall=latency
    emu_case "latency timer stalls" --stderr --expect-fail --filter "$O" -- -d $D --samples 10
    scanaplus_options stall=read-eeprom
    emu_case "EEPROM read stalls" --stderr --expect-fail --filter "$O" -- -d $D --samples 10
    scanaplus_options

    # Unplugs, counted in the host's transfers. After one, libusb's
    # release fails and the C driver logs "Failed to close FTDI device
    # (-1): " with libftdi's stale error string, always in "unplug
    # before EEPROM word 17" and "unplug mid capture". oxrok cannot show
    # it: nusb's release cannot fail (nusb 0.2.8,
    # linux_usbfs/device.rs:721-731), a real difference, so the line is
    # not compared in those cases. In "unplug after the scan" the C
    # driver logs it only when the removal gets ahead of the scan's
    # release. The "unplug before write N" cases never log it: sigrok-cli
    # exits at g_critical("Failed to start session.") (session.c:896-897,
    # main.c:46-47), so dev_close never runs.
    X='/^Failed to close FTDI device /d'
    # Endpoint 0's control OUTs: the scan's open sends 2, so the device
    # is gone when dev_open opens it again. Which step of
    # ftdi_usb_open_desc fails depends on the removal's timing: the
    # product string with the node still there (-8), libusb_open() with
    # the node gone and the device still in libusb's list (-4), or no
    # device in the list (-3).
    GONE="$O; $X; s/^Failed to open device \\((-3|-4|-8)\\): .*/Failed to open device (gone)./"
    emu_case "unplug after the scan" --unplug out=0:2 --stderr --expect-fail --filter "$GONE" -- -d $D --samples 10
    # Control reads: 2 string reads by the scan's open, 2 by dev_open's,
    # then EEPROM words 16 and 17.
    emu_case "unplug before EEPROM word 17" --unplug in=0:5 --stderr --expect-fail --filter "$O; $X" -- -d $D --samples 10
    # The first write of scanaplus_init() has no bulk OUT before it to
    # count: the unplug follows the 6th control read, EEPROM word 17,
    # the last transfer of dev_open.
    emu_case "unplug before write 1" --unplug in=0:6 --stderr --expect-fail --filter "$S" -- -d $D --samples 10
    # Each other write of scanaplus_init() and
    # scanaplus_start_acquisition() with its own error return: write N
    # fails after N-1 bulk OUTs (out=2:N-1).
    # 2 to 5 the init's head, 6 and 7 the first 0x8d pulse, 120 to 122
    # the init's id, 123 its last, 124 and 125 the start's thresholds
    # and configuration, 126 to 128 the cleared id, 129 the id again.
    for n in 2 3 4 5 6 7 120 121 122 123 124 125 126 127 128 129; do
        emu_case "unplug before write $n" --unplug out=2:$((n - 1)) --stderr --expect-fail --filter "$S" -- -d $D --samples 10
    done
    # Mid capture: the third bulk IN of the dropped read.
    emu_case "unplug mid capture" --unplug in=1:3 --stderr --filter "$N; $X" -- -d $D --samples 100 -O hex
}
