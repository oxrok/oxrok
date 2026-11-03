# A ChronoVu LA8 or LA16 (emu/src/models/chronovu-la.rs): an FTDI chip in
# FIFO mode in front of 8 MiB of sample memory. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a device model".
#
# The driver ignores the sample limit: every capture reads and sends the
# whole memory (8 Mi samples on the LA8, 4 Mi on the LA16), and
# sigrok-cli hands every packet to the output module, so a capture's
# output is tens of MB. Captures keep their first lines (the header, a
# trigger marker, the first 192 samples of each channel) and end with
# the SHA-256 of the whole output (--digest); the transactions hold a
# digest of every transfer of the readout.
EMU_MODEL=chronovu-la
EMU_OPTIONS=""
EMU_DRIVER=chronovu-la
EMU_GCOV_SHARED="src/usb.c"
# oxrok has no chronovu-la driver yet: these are oracle cases.
EMU_FULL=""

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change. None restores EMU_OPTIONS.
chronovu_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "chronovu-la: the emulator refused '$*'"
}

emu_cases() {
    D=chronovu-la
    # The header, the trigger marker if any, one line per channel.
    HEAD8='11,$d'
    HEAD16='19,$d'
    # sigrok-cli's own lines after the driver's: a repeated line once.
    UNIQ='$!N; /^(.*)\n\1$/!P; D'
    # The harness makes the first attach's address generic; a device
    # attached again for other options has a new one.
    ADDR='s/conn=[0-9]+\.[0-9]+/conn=BUS.ADDR/; s/conn: [0-9]+\.[0-9]+/conn: BUS.ADDR/'

    # The LA8, at 0403:8867 (the udev rules' "new VID/PID").
    emu_case "la8 scan" --keep-header --filter "$ADDR" -- -d $D --scan
    emu_case "la8 show" --keep-header --filter "$ADDR" -- -d $D --show
    emu_case "la8 get samplerate" -- -d $D --get samplerate
    # conn is the device's bus.address, which changes per attach.
    emu_case "la8 get conn" --filter "s/'[0-9]+\\.[0-9]+'/'BUS.ADDR'/" -- -d $D --get conn
    emu_case "la8 set samplerate" --no-output -- -d $D --set -c samplerate=50m
    # 100 MHz is divcount 0, where the two bytes of a pair come in
    # sample order; the sample limit is set but not used.
    emu_case "la8 100 MHz" --digest --filter "$HEAD8" -- -d $D --samples 100 -O hex
    # A state trigger: the memory starts at the first sample with
    # channel 7 high, 128 periods after the command.
    emu_case "la8 trigger" --digest --filter "$HEAD8" -- -d $D -c samplerate=1m --triggers 7=1 --samples 100 -O hex
    # The slowest rate (divcount 0xfe); LIMIT_MSEC is taken and unused.
    # That this capture completes is an emulation artifact (the memory
    # fills in polls): on a real LA8, 8 Mi samples take 21 s here, and
    # the driver's deadline (api.c:491, the capture time added in
    # seconds to a microsecond clock) is 10 s, so it would report
    # "Trigger timed out. Bytes read: 0." from divcount 119 on.
    emu_case "la8 slowest" --digest --filter "$HEAD8" -- -d $D -c samplerate=392156 --time 5 -O hex
    # Three channels; a trigger of two (sigrok-cli refuses one on a
    # disabled channel), pattern 0x02, mask 0x03: sample 2.
    emu_case "la8 channels trigger by conn" --digest --filter '6,$d' -- \
        -d $D:conn=$EMU_USB -C 0,1,4 -c samplerate=20m --triggers 0=0,1=1 --samples 100 -O hex
    # conn= by ids, and two that sr_usb_find refuses: an address beyond
    # 127, no numbers at all.
    emu_case "la8 by ids" --keep-header --filter "$ADDR" -- -d $D:conn=0403.8867 --scan
    emu_case "la8 conn out of range" --stderr --expect-fail -- -d $D:conn=1.200 --samples 10
    emu_case "la8 conn zero" --stderr --expect-fail -- -d $D:conn=0.0 --samples 10
    emu_case "la8 get limit" --stderr --expect-fail -- -d $D --get limit_samples
    # The error answers before any write.
    emu_case "la8 samplerate invalid" --stderr --expect-fail -- -d $D -c samplerate=3m --samples 10
    emu_case "la8 too many samples" --stderr --expect-fail -- -d $D --samples 9m
    emu_case "la8 edge trigger" --stderr --expect-fail -- -d $D --samples 10 --triggers 0=r

    # With 100 samples kept before the trigger sample (an option: the
    # sources do not say whether the device keeps any), the driver's
    # search finds the trigger at sample 100 of block 0 and splits the
    # block there (protocol.c:482-492).
    chronovu_options pretrigger=100
    emu_case "la8 pretrigger" --digest --filter "$HEAD8" -- -d $D -c samplerate=1m --triggers 7=1 --samples 100 -O hex

    # The LA16.
    chronovu_options model=la16
    emu_case "la16 scan" --keep-header --filter "$ADDR" -- -d $D --scan
    emu_case "la16 show" --keep-header --filter "$ADDR" -- -d $D --show
    emu_case "la16 200 MHz" --digest --filter "$HEAD16" -- -d $D --samples 100 -O hex
    emu_case "la16 slowest channels" --digest --filter '7,$d' -- \
        -d $D -C 0,8,15 -c samplerate=784313 --samples 100 -O hex
    # Edge triggers are the LA16's: channel 1 rising with channel 0 low
    # is sample 2. The driver's search only sees state bits (mask 0x0001,
    # pattern 0): sample 0's first byte already matches.
    emu_case "la16 rising" --digest --filter "$HEAD16" -- -d $D --triggers 1=r,0=0 --samples 100 -O hex
    # An edge alone leaves the driver's trigger mask 0, and its search
    # stops at once (protocol.c:436): no trigger marker, though the
    # device triggered (the data start at sample 2).
    emu_case "la16 edge only" --digest --filter "$HEAD16" -- -d $D --triggers 1=r --samples 100 -O hex
    # The driver's trigger search compares single bytes, before its
    # byte swap, with the 16 bit mask and an 8 bit expected value: for
    # channels 8 to 15 both sides are 0, so the first byte matches
    # whatever the data (here where the device triggered, sample 0).
    emu_case "la16 trigger channel 9" --digest --filter "$HEAD16" -- -d $D --triggers 9=1 --samples 100 -O hex
    # Channel 0 high matches the second byte of sample 0: the driver
    # sends a 1 byte packet of unitsize 2 before the trigger and the
    # rest of the block a byte off. csv drops the odd byte and shows the
    # block's samples shifted ...
    emu_case "la16 trigger csv" --digest --filter '10,$d' -- -d $D --triggers 0=1 --samples 100 -O csv:header=false
    # ... while hex (and bits) takes length - unitsize, 1 - 2 in 64 bits,
    # as its loop bound and reads past the packet until it crashes.
    emu_case "la16 trigger hex crashes" --stderr --expect-fail \
        --filter '/Trigger pattern/!d; s/^.*chronovu-la: //' -- -l 4 -d $D --triggers 0=1 --samples 100 -O hex

    # The variants: the "old VID/PID" (ftdi_sio binds it at attach and
    # libftdi detaches it), an FT232R, a BM without serial number
    # (bcdDevice 0x0200, which libftdi still takes for a BM), and
    # product strings the driver does not know.
    chronovu_options pid=0x6001
    emu_case "pid 6001 scan" --keep-header --filter "$ADDR" -- -d $D --scan
    emu_case "pid 6001 capture" --digest --filter "$HEAD8" -- -d $D -c samplerate=10m --triggers 3=1 --samples 100 -O hex
    chronovu_options chip=r
    emu_case "ft232r scan" --keep-header --filter "$ADDR" -- -d $D --scan
    chronovu_options serial=
    emu_case "no serial scan" --keep-header --filter "$ADDR" -- -d $D --scan
    chronovu_options product=LA8
    emu_case "unknown product" --stderr --expect-fail -- -d $D --samples 10
    chronovu_options product=
    emu_case "no product" --stderr --expect-fail -- -d $D --samples 10

    # Faults. The inputs never meet the trigger: the device answers
    # status-only reads for ever and the driver's block 0 loop keeps
    # reading. The harness stops sigrok-cli after 3 s (--hangs), before
    # the driver's own 10 s deadline, so this shows the device never
    # triggering, not the driver's timeout path; the transactions are
    # cut where the status-only reads start repeating.
    chronovu_options no-trigger=1
    emu_case "no trigger" --hangs --timeout 3 --stderr --no-output -- -d $D --triggers 0=1 --samples 10
    chronovu_options
    # The FIFO runs dry for one read in the middle of block 0, then of
    # block 2: libftdi returns what it has, the driver takes it for a
    # trigger timeout and resets the device.
    chronovu_options gap=1
    emu_case "gap in block 0" --stderr --filter "$UNIQ" -- -d $D --samples 100 -O hex
    chronovu_options gap=3
    emu_case "gap in block 2" --stderr --filter "$UNIQ" -- -d $D --samples 100 -O hex
    # The bulk IN endpoint halts: the read fails.
    chronovu_options in-stall=4
    emu_case "read error" --stderr --filter "$UNIQ" -- -d $D --samples 100 -O hex
    # The requests of the open, each STALLed: libftdi's reset in
    # ftdi_usb_open_desc, its 9600 Bd, the purges, the flow control.
    chronovu_options stall=reset/0
    emu_case "open reset stalls" --stderr --expect-fail -- -d $D --samples 10
    chronovu_options stall=baud
    emu_case "open baud stalls" --stderr --expect-fail -- -d $D --samples 10
    chronovu_options stall=reset/1
    emu_case "purge stalls" --stderr --expect-fail -- -d $D --samples 10
    chronovu_options stall=flow-ctrl
    emu_case "flow control stalls" --stderr --expect-fail -- -d $D --samples 10
    chronovu_options
    # Unplugged during the readout: the read fails, and the sequencer
    # reset's write fails, which calls the sequencer reset again, for
    # ever (cv_write and close_usb_reset_sequencer): a stack overflow.
    emu_case "unplug in readout" --unplug in=1:10 --stderr --expect-fail --filter "$UNIQ" -- -d $D --samples 100 -O hex
    # Unplugged right after the last transfer of the readout (2 status-only
    # reads, then 2115): the capture is complete, the close fails.
    emu_case "unplug after readout" --unplug in=1:2117 --stderr --filter "$UNIQ" -- -d $D --samples 100 -O hex
}
