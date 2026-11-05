# A ChronoVu LA8 or LA16 (emu/src/models/chronovu-la.rs): an FTDI chip in
# FIFO mode in front of 8 MiB of sample memory. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "Writing a device model".
#
# libsigrok's driver ignores the sample limit: every capture reads and
# sends the whole memory (8 Mi samples on the LA8, 4 Mi on the LA16),
# and sigrok-cli hands every packet to the output module, so a capture's
# output is tens of MB. oxrok's port reads the whole memory too (the
# samples are spread over all of it) but sends no more than the limit,
# so a capture with a smaller limit is an --oracle-differs case, and
# the "whole memory" cases compare every sample of both tools. Captures
# keep their first lines (the header, a trigger marker, the first 192
# samples of each channel) and end with the SHA-256 of the whole output
# (--digest); the transactions hold a digest of every transfer of the
# readout.
EMU_MODEL=chronovu-la
EMU_OPTIONS=""
EMU_DRIVER=chronovu-la
EMU_GCOV_SHARED="src/usb.c"
# oxrok has the driver (src/drivers/chronovu_la): every case runs both
# tools. As with ftdi-la, sigrok-cli runs first in each and libftdi
# leaves ftdi_sio detached from port A, so oxrok finds it free; oxrok
# gives it back when it closes.
EMU_FULL=chronovu-la
# The memory readouts are thousands of same-shape bulk INs: fold them
# (tools/diffcli.sh, EMU_TX_FOLD).
EMU_TX_FOLD=1

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change. None restores EMU_OPTIONS.
chronovu_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "chronovu-la: the emulator refused '$*'"
}

emu_cases() {
    D=chronovu-la
    # The header, the trigger marker if any, one line per channel.
    HEAD8='11,$d'
    HEAD16='19,$d'
    # oxrok's log lines and error texts as sigrok-cli words them: its log
    # prefix (the conn parser's lines are libsigrok's usb.c), and the
    # sr_strerror() texts for its errors, whose cause the driver's own
    # line gives. Written for a pattern space of one or two lines, so
    # that it runs before UNIQ compares them.
    E='s/(^|\n)oxrok: [a-z]+: oxrok::drivers::chronovu_la(::protocol)?: (Invalid input|Could neither)/\1sr: usb: \3/g
s/(^|\n)oxrok: [a-z]+: oxrok::drivers::chronovu_la(::protocol)?: /\1sr: chronovu-la: /g
s/(^|\n)(Failed to open device): [^\n]*/\1\2./g
s/(^|\n)(Failed to set device option .samplerate.): device error: [^\n]*/\1\2: generic\/unspecified error./g
s/(^|\n)(Failed to get .limit_samples.): unknown option: limit_samples/\1\2: not applicable/g'
    # sigrok-cli's own lines after the driver's: a repeated line once.
    UNIQ='$!N; '"$E"'
/^(.*)\n\1$/!P; D'
    # The harness makes the first attach's address generic; a device
    # attached again for other options has a new one.
    ADDR='s/conn=[0-9]+\.[0-9]+/conn=BUS.ADDR/; s/conn: [0-9]+\.[0-9]+/conn: BUS.ADDR/'
    # The libsigrok bugs the port does not copy (docs/DEVICES.md).
    LIMIT="api.c:337-342 stores limit_samples and limit_msec, which nothing reads: the whole memory goes out"
    BYTES="protocol.c:425-445 compares single bytes, before the LA16 byte swap, with the 16 bit mask and an 8 bit expected value"
    EDGES="protocol.c:436 searches for no trigger when the state mask is 0, as for an edge-only trigger"
    RECURSE="protocol.c:137-145 and 198: a failed write resets the sequencer, whose own write fails and resets it again until the stack overflows"
    WAIT="api.c:491-492 waits 10 s of wall time, so sigrok-cli makes as many reads as the machine manages, where oxrok makes a fixed count: the answer at this rate is the same, only the poll count differs"

    # The LA8, at 0403:8867 (the udev rules' "new VID/PID").
    emu_case "la8 scan" --keep-header --filter "$ADDR" -- -d $D --scan
    emu_case "la8 show" --keep-header --filter "$ADDR" -- -d $D --show
    emu_case "la8 get samplerate" -- -d $D --get samplerate
    # conn is the device's bus.address, which changes per attach.
    emu_case "la8 get conn" --filter "s/'[0-9]+\\.[0-9]+'/'BUS.ADDR'/" -- -d $D --get conn
    emu_case "la8 set samplerate" --no-output -- -d $D --set -c samplerate=50m
    # 100 MHz is divcount 0, where the two bytes of a pair come in
    # sample order; libsigrok sends the whole memory for 100 samples.
    emu_case "la8 100 MHz" --oracle-differs "$LIMIT" --digest --filter "$HEAD8" -- -d $D --samples 100 -O hex
    # A state trigger: the memory starts at the first sample with
    # channel 7 high, 128 periods after the command.
    emu_case "la8 trigger" --oracle-differs "$LIMIT" --digest --filter "$HEAD8" -- -d $D -c samplerate=1m --triggers 7=1 --samples 100 -O hex
    emu_case "la8 100 MHz whole memory" --digest --filter "$HEAD8" -- -d $D --samples 8388608 -O hex
    # The same with the whole memory as the limit: every sample of both
    # tools compared.
    emu_case "la8 whole memory" --digest --filter "$HEAD8" -- -d $D -c samplerate=1m --triggers 7=1 --samples 8388608 -O hex
    # The slowest rate (divcount 0xfe); LIMIT_MSEC is taken, and used by
    # oxrok only (5 ms, 1960 samples).
    # That this capture completes is an emulation artifact (the memory
    # fills in polls): on a real LA8, 8 Mi samples take 21 s here, and
    # libsigrok's deadline (api.c:491-492, the capture time added in seconds
    # to a microsecond clock) is 10 s, so it would report "Trigger timed
    # out. Bytes read: 0." from divcount 119 on. oxrok waits 10 s plus
    # the capture time, counted in status-only reads at least 16 ms apart.
    emu_case "la8 slowest" --oracle-differs "$LIMIT" --digest --filter "$HEAD8" -- -d $D -c samplerate=392156 --time 5 -O hex
    # Three channels; a trigger of two (sigrok-cli refuses one on a
    # disabled channel), pattern 0x02, mask 0x03: sample 2.
    emu_case "la8 channels trigger by conn" --oracle-differs "$LIMIT" --digest --filter '6,$d' -- \
        -d $D:conn=$EMU_USB -C 0,1,4 -c samplerate=20m --triggers 0=0,1=1 --samples 100 -O hex
    # conn= by ids, and two that sr_usb_find refuses: an address beyond
    # 127, no numbers at all.
    emu_case "la8 by ids" --keep-header --filter "$ADDR" -- -d $D:conn=0403.8867 --scan
    emu_case "la8 conn out of range" --stderr --expect-fail --filter "$E" -- -d $D:conn=1.200 --samples 10
    emu_case "la8 conn zero" --stderr --expect-fail --filter "$E" -- -d $D:conn=0.0 --samples 10
    emu_case "la8 get limit" --stderr --expect-fail --filter "$E" -- -d $D --get limit_samples
    # The error answers before any write.
    emu_case "la8 samplerate invalid" --stderr --expect-fail --filter "$E" -- -d $D -c samplerate=3m --samples 10
    emu_case "la8 too many samples" --stderr --expect-fail -- -d $D --samples 9m
    emu_case "la8 edge trigger" --stderr --expect-fail -- -d $D --samples 10 --triggers 0=r
    # The device unplugged after the open's last request (flow control,
    # the fifth control write): the command's write fails, and
    # libsigrok's sequencer reset writes, fails and recurses until
    # sigrok-cli dies. oxrok resets once (the write, purge and reset
    # each fail) and refuses the start.
    emu_case "unplug at the command" --oracle-bug "$RECURSE" --unplug out=0:5 --stderr --expect-fail --filter "$UNIQ" -- -d $D --samples 100 -O hex

    # With 100 samples kept before the trigger sample (an option: the
    # sources do not say whether the device keeps any), the driver's
    # search finds the trigger at sample 100 of block 0 and splits the
    # block there (protocol.c:482-492).
    chronovu_options pretrigger=100
    emu_case "la8 pretrigger" --oracle-differs "$LIMIT" --digest --filter "$HEAD8" -- -d $D -c samplerate=1m --triggers 7=1 --samples 100 -O hex
    emu_case "la8 pretrigger whole memory" --digest --filter "$HEAD8" -- -d $D -c samplerate=1m --triggers 7=1 --samples 8388608 -O hex

    # The LA16.
    chronovu_options model=la16
    emu_case "la16 scan" --keep-header --filter "$ADDR" -- -d $D --scan
    emu_case "la16 show" --keep-header --filter "$ADDR" -- -d $D --show
    emu_case "la16 200 MHz" --oracle-differs "$LIMIT" --digest --filter "$HEAD16" -- -d $D --samples 100 -O hex
    emu_case "la16 slowest channels" --oracle-differs "$LIMIT" --digest --filter '7,$d' -- \
        -d $D -C 0,8,15 -c samplerate=784313 --samples 100 -O hex
    # Edge triggers are the LA16's: channel 1 rising with channel 0 low
    # is sample 2, where the memory starts. libsigrok's search only sees
    # state bits (mask 0x0001, pattern 0), and sample 0's first byte
    # matches; oxrok finds sample 0 with channel 1 high and channel 0
    # low, the edge before the memory.
    emu_case "la16 rising" --oracle-differs "$LIMIT" --digest --filter "$HEAD16" -- -d $D --triggers 1=r,0=0 --samples 100 -O hex
    emu_case "la16 whole memory" --digest --filter "$HEAD16" -- -d $D --triggers 1=r,0=0 --samples 4194304 -O hex
    # An edge alone leaves the driver's trigger mask 0, and its search
    # stops at once (protocol.c:436): no trigger marker, though the
    # device triggered (the data start at sample 2). oxrok marks it.
    emu_case "la16 edge only" --oracle-differs "$EDGES; $LIMIT" --digest --filter "$HEAD16" -- -d $D --triggers 1=r --samples 192 -O hex
    # libsigrok's trigger search compares single bytes, before its byte
    # swap, with the 16 bit mask and an 8 bit expected value: for
    # channels 8 to 15 both sides are 0, so the first byte matches
    # whatever the data (here where the device triggered, sample 0, as
    # oxrok finds it).
    emu_case "la16 trigger channel 9" --oracle-differs "$LIMIT" --digest --filter "$HEAD16" -- -d $D --triggers 9=1 --samples 100 -O hex
    # Channel 0 high matches the second byte of sample 0: libsigrok
    # sends a 1 byte packet of unitsize 2 before the trigger and the
    # rest of the block a byte off. csv drops the odd byte and shows the
    # block's samples shifted ...
    emu_case "la16 trigger csv" --oracle-differs "$BYTES; $LIMIT" --digest --filter '10,$d' -- -d $D --triggers 0=1 --samples 100 -O csv:header=false
    # ... while hex (and bits) takes length - unitsize, 1 - 2 in 64 bits,
    # as its loop bound and reads past the packet until it crashes (the
    # command, pattern and mask 0x0001, is in the transactions). oxrok's
    # trigger is sample 0, and the capture ends. Not at -l 4: the
    # .oracle golden keeps sigrok-cli's stderr, timestamps and all.
    emu_case "la16 trigger hex crashes" --oracle-bug "$BYTES" --digest --filter "$HEAD16" -- -d $D --triggers 0=1 --samples 100 -O hex
    # The device keeps 100 samples before the trigger: channel 1 rising
    # alone is sample 2 of the memory. libsigrok sends no trigger. 192
    # samples, a full line: hex prints its T: line only with one.
    chronovu_options model=la16 pretrigger=100
    emu_case "la16 rising pretrigger" --oracle-differs "$EDGES; $LIMIT" --digest --filter "$HEAD16" -- -d $D --triggers 1=r --samples 192 -O hex
    # The limit of oxrok's rule that an edge channel at its new level on
    # sample 0 is the edge: channel 1 falls at sample 4, but is low from
    # sample 0, so oxrok marks sample 0. Nothing in the data tells this
    # from a device that kept no samples before its trigger.
    emu_case "la16 falling pretrigger" --oracle-differs "$EDGES; $LIMIT" --digest --filter "$HEAD16" -- -d $D --triggers 1=f --samples 192 -O hex

    # The variants: the "old VID/PID" (ftdi_sio binds it at attach and
    # libftdi detaches it), an FT232R, a BM without serial number
    # (bcdDevice 0x0200, which libftdi still takes for a BM), and
    # product strings the driver does not know.
    chronovu_options pid=0x6001
    emu_case "pid 6001 scan" --keep-header --filter "$ADDR" -- -d $D --scan
    emu_case "pid 6001 capture" --oracle-differs "$LIMIT" --digest --filter "$HEAD8" -- -d $D -c samplerate=10m --triggers 3=1 --samples 100 -O hex
    chronovu_options chip=r
    emu_case "ft232r scan" --keep-header --filter "$ADDR" -- -d $D --scan
    chronovu_options serial=
    emu_case "no serial scan" --keep-header --filter "$ADDR" -- -d $D --scan
    chronovu_options product=LA8
    emu_case "unknown product" --stderr --expect-fail -- -d $D --samples 10
    chronovu_options product=
    emu_case "no product" --stderr --expect-fail -- -d $D --samples 10

    # Faults. The inputs never meet the trigger: the device answers
    # status-only reads for ever. Both tools give up after about 10 s
    # (oxrok after 10 s plus the capture time, 84 ms at 100 MHz, in reads
    # at least 16 ms apart, 631 after the first), report the timeout and
    # reset the device. This takes 20 s or so of the session.
    chronovu_options no-trigger=1
    emu_case "no trigger" --oracle-differs "$WAIT" --no-output -- -d $D --triggers 0=1 --samples 10
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
    # The same, recorded for the replay tests (the whole readout would
    # be 17 MB of fixture): the open, the command, the polls, a block,
    # the failed read and the reset; nothing on stdout.
    emu_case "read error record" --record --no-output -- -d $D -c samplerate=1m --triggers 7=1 --samples 100 -O hex
    chronovu_options model=la16 in-stall=4
    emu_case "la16 read error record" --record --no-output -- -d $D --triggers 1=r,0=0 --samples 100 -O hex
    # The requests of the open, each STALLed: libftdi's reset in
    # ftdi_usb_open_desc, its 9600 Bd, the purges, the flow control.
    chronovu_options stall=reset/0
    emu_case "open reset stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    chronovu_options stall=baud
    emu_case "open baud stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    chronovu_options stall=reset/1
    emu_case "purge stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    chronovu_options stall=flow-ctrl
    emu_case "flow control stalls" --stderr --expect-fail --filter "$E" -- -d $D --samples 10
    chronovu_options
    # Unplugged during the readout: the read fails, and the sequencer
    # reset's write fails, which calls the sequencer reset again, for
    # ever (cv_write and close_usb_reset_sequencer): a stack overflow.
    # oxrok logs the failed write, purge and reset once each, and no
    # failed close: nusb 0.2.8 cannot report a failed release
    # (linux_usbfs/device.rs:721-731).
    emu_case "unplug in readout" --oracle-bug "$RECURSE" --unplug in=1:10 --stderr --filter "$UNIQ" -- -d $D --samples 100 -O hex
    # Unplugged right after the last transfer of the readout (2 status-only
    # reads, then 2115): the capture is complete, and the close fails.
    # libsigrok logs "Failed to close FTDI device (-1): all fine." (libusb's
    # release fails); nusb 0.2.8 cannot report a failed release
    # (linux_usbfs/device.rs:721-731), so the case compares the samples.
    # A SIGINT while the samples go out: oxrok's stop must end the run
    # though the device's thread may be blocked on a full queue.
    # sigrok-cli has no SIGINT handler.
    emu_case "interrupt after readout" --oracle-bug "sigrok-cli has no SIGINT handler" --interrupt in=1:2117 --stderr --filter 's/sent [0-9]+ samples/sent N samples/' -- -d $D --samples 8388608 -O hex
    emu_case "unplug after readout" --oracle-differs "$LIMIT" --unplug in=1:2117 --digest --filter "$HEAD8" -- -d $D --samples 100 -O hex
}
