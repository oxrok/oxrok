# The ASIX SIGMA (emu/src/models/asix-sigma/): an FT245BM in front of a
# Xilinx FPGA that the driver configures in bit-bang mode on every open
# of a capture, then drives through register commands and a sample
# memory readout. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a device model". oxrok has the driver (src/drivers/asix_sigma):
# every case runs both tools.
#
# How a capture ends decides which cases can exist. The model's time
# runs one 10 ms period per position poll (the driver's timer), but the
# driver ends a capture on wall time: its time limit (the sample count
# turned into ms, plus twice 65536 sample periods) is checked at each
# poll. Without a trigger that limit is only certain to have run out at
# the first poll when it is well below the timer's 10 ms: rates of 25
# MHz and up with up to 150000 samples, or 100/200 MHz. With a trigger
# the limit starts at the poll that sees it and is the post trigger
# share of the count: again 25 MHz and up. captureratio=100 leaves no
# limit at all, so the capture ends when the device says so: any rate.
EMU_MODEL=asix-sigma
EMU_DRIVER=asix-sigma
EMU_GCOV_SHARED="src/usb.c"
EMU_FULL=asix-sigma
# sigrok-cli runs first in each case, and libftdi detaches the kernel's
# ftdi_sio without attaching it again; oxrok gives the chip back to the
# kernel when it closes it, which no case shows (see ftdi-la.sh).
# The memory readouts are thousands of same-shape bulk INs: fold them
# (tools/diffcli.sh, EMU_TX_FOLD).
EMU_TX_FOLD=1

# The model's options for the cases that follow (none: the defaults).
asix_sigma_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "asix-sigma: the emulator refused '$*'"
}

emu_cases() {
    D=asix-sigma
    # A long output keeps its first 16 and its last line (--digest
    # holds the rest).
    LONG='17,$ {$!d}'
    # The session names the device by its connection, the bus address.
    A='s/device [0-9]+[.][0-9]+ /device BUS.ADDR /'
    # oxrok's log lines and error texts as sigrok-cli words them: its
    # log prefix (sr_usb_find()'s messages are libsigrok's usb.c's), the
    # sr_strerror() texts, and the tool's own line on a failed open,
    # which oxrok follows with the reason.
    E='s/^oxrok: [a-z]+: oxrok::drivers::asix_sigma: (Could neither|Invalid input)/sr: usb: \1/
s/^oxrok: [a-z]+: oxrok::drivers::asix_sigma(::protocol)?: /sr: asix-sigma: /
s/^(Failed to set device option .[a-z_]+.: invalid argument): .*$/\1./
s/^(Failed to get .[a-z_]+.): unknown option: .*$/\1: not applicable/
s/^(Failed to open device).*$/\1./'
    # A capture that fails to start: the driver's messages, then
    # libsigrok's session (sigrok-cli's two "sr: session:" lines and its
    # "Failed to start session.") or oxrok's session (the driver's error,
    # which sigrok-cli never prints). Neither tool's last lines have a
    # counterpart in the other: they are reduced to where the run failed.
    F="$E
/^sr: session: /d
\$s/.*/start refused/"
    # The bugs of the C driver that oxrok does not copy, whose cases are
    # --oracle-differs: sigrok-cli's output is kept as SLUG.oracle.
    # A wrapped memory: alloc_sample_buffer() stops at the start of the
    # row before the one being written (protocol.c:1418-1421), so
    # protocol.c:2065 decodes nothing of it.
    WRAP="protocol.c:1418-1421 end a wrapped readout at the start of its last row, of which protocol.c:2065 decodes nothing"
    # rewind_trig_arm_pos() stops at the start position (protocol.c:1387),
    # and sigma_location_check() compares the position with it only after
    # an event moved past it (protocol.c:1913, 1775): a trigger in the
    # first 28 events gets no marker.
    START="protocol.c:1387 arms the trigger check at the start position, which protocol.c:1775 compares only after an event's increment (protocol.c:1913)"
    # By the address of the session's first attach, before any case
    # changes the descriptors.
    emu_case "by address" -- -d $D:conn=$EMU_USB -c samplerate=50m --samples 20 -O bits
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "by ids" --keep-header -- -d $D:conn=a600.a000 --scan
    # conn= strings sr_usb_find rejects.
    for c in nonsense 300.1 1.200 0.0; do
        emu_case "bad conn $c" --stderr --filter "$E" -- -d $D:conn=$c --scan
    done
    emu_case "show" --keep-header --filter 's/conn: [0-9]+[.][0-9]+/conn: BUS.ADDR/' -- -d $D --show
    emu_case "probe names" --keep-header -- -d $D:probe_names=a,b,c --scan
    for k in samplerate captureratio external_clock external_clock_source clock_edge \
        limit_samples limit_time; do
        emu_case "get $k" -- -d $D --get $k
    done
    emu_case "get conn" --filter "s/'[0-9]+[.][0-9]+'/'BUS.ADDR'/" -- -d $D --get conn
    emu_case "set samplerate" --no-output -- -d $D --set -c samplerate=1m
    emu_case "set bad samplerate" --stderr --expect-fail --filter "$E
/Failed to set/!d" -- -d $D --set -c samplerate=100k
    emu_case "set clock source" --no-output -- -d $D --set -c external_clock_source=7
    emu_case "set bad clock edge" --stderr --expect-fail --filter "$E
/Failed to set/!d" -- -d $D --set -c clock_edge=sideways
    emu_case "get unsupported" --stderr --expect-fail --filter "$E" -- -d $D --get voltage_threshold
    emu_case "set bad clock source" --stderr --expect-fail --filter "$E
/Failed to set/!d" -- -d $D --set -c external_clock_source=17

    # Captures that end at the first poll on the driver's time limit,
    # then a forced stop.
    emu_case "50 MHz" -- -d $D -c samplerate=50m --samples 100 -O hex
    emu_case "25 MHz" -- -d $D -c samplerate=25m --samples 100 -O hex
    # Rounded down to 50 MHz / 2.
    emu_case "30 MHz" -- -d $D -c samplerate=30m --samples 100 -O hex
    emu_case "100 MHz" -- -d $D -c samplerate=100m --samples 100 -O hex
    emu_case "200 MHz" -- -d $D -c samplerate=200m --samples 100 -O hex
    emu_case "channels" -- -d $D -c samplerate=50m -C 1,4,16 --samples 64 -O bits
    emu_case "time" --digest --filter "$LONG" -- -d $D -c samplerate=50m --time 2 -O hex
    emu_case "150000 samples" --digest --filter "$LONG" -- -d $D -c samplerate=50m --samples 150000 -O hex
    # A time limit long enough for the memory to fill (14.7 million
    # events, 30 polls at 50 MHz) well before it: without a trigger the
    # driver stops once the write position wraps and reads all rows but
    # the one being written, 32767, yet decodes only 32766 of them: the
    # stop position it computes for a wrapped memory is the start of the
    # last row, so that row's event count (stop & ROW_MASK) is 0.
    # memory but the row being written.
    emu_case "memory wraps" --oracle-differs "$WRAP" --digest --filter "$LONG" -- -d $D -c samplerate=50m --time 10000 -C 1 -O hex
    # External clock: the driver takes it as 50 MHz. Channel 16 as the
    # clock writes 16 (api.c:578) into the 4 bit pin field (protocol.h:137),
    # which sets CLKSEL_RISING: oxrok writes the same, and warns on stderr.
    emu_case "external clock" -- -d $D -c external_clock=1 --samples 50 -O hex
    emu_case "external clock falling" -- -d $D -c external_clock=1:external_clock_source=3:clock_edge=falling --samples 50 -O hex
    emu_case "external clock 16 rising" -- -d $D -c external_clock=1:external_clock_source=16 --samples 50 -O hex
    emu_case "external clock either" -- -d $D -c external_clock=1:external_clock_source=16:clock_edge=either --samples 50 -O hex
    emu_case "rate too high" --stderr --expect-fail --filter "$E" -- -d $D -c samplerate=60m --samples 10

    # With a trigger the driver submits every sample it downloads,
    # whatever --samples says (protocol.c, addto_submit_buffer), and
    # sigrok-cli hands them all to the output module: keep the first
    # block of 64 samples and its trigger marker, and the digest.
    T='20,$ {$!d}'
    # The same with two blocks, and three.
    T2='36,$ {$!d}'
    T3='52,$ {$!d}'
    # A trigger in the 50 MHz netlist that ends on the time limit: the
    # poll that sees the trigger starts the limit, the next ends the
    # capture with a forced stop (2 polls, about 2200 rows).
    emu_case "trigger level" --digest --filter "$T" -- -d $D -c samplerate=50m --samples 40 -t 5=1 -O bits
    # The others with captureratio=100: no limit, the device stops 1/256
    # of the memory (128 rows) after the trigger.
    R=captureratio=100
    emu_case "trigger rising" --digest --filter "$T" -- -d $D -c samplerate=50m:$R --samples 40 -t 5=r -O bits
    emu_case "trigger falling" --digest --filter "$T" -- -d $D -c samplerate=25m:$R --samples 40 -t 5=f -O bits
    emu_case "trigger level and edge" --digest --filter "$T" -- -d $D -c samplerate=50m:$R --samples 60 -t 1=0,4=0,5=r -O bits
    # A trigger within 28 events of the start gets no marker: the
    # software check is armed at the start position, which the decode
    # compares only after moving past it.
    emu_case "trigger near the start" --oracle-differs "$START" --digest --filter "$T" -- -d $D -c samplerate=50m:$R --samples 40 -t 3=r -O bits
    # The trigger 28 events before the hardware's position lies in the
    # previous row: the rewind crosses a row boundary. The marker comes
    # after the eighth block (samples 448 to 511): keep the first block
    # and the T: line.
    emu_case "trigger in row 1" --digest --filter '18,$ {/^T:/!d}' -- -d $D -c samplerate=50m:captureratio=100 --samples 40 -t 6=1,7=1,8=1 -O bits
    # sigrok-cli refuses a trigger on a disabled channel itself.
    emu_case "trigger on a disabled channel" --stderr --expect-fail --filter "$A" -- -d $D -c samplerate=50m:captureratio=100 -C 1,2 --samples 40 -t 5=r -O bits
    emu_case "trigger two edges" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 40 -t 1=r,2=f
    # No limit with captureratio=100: the device stops on its own.
    emu_case "trigger ratio 100" --digest --filter "$T" -- -d $D -c samplerate=50m:captureratio=100 --samples 40 -t 5=1 -O bits
    emu_case "trigger 10 MHz" --digest --filter "$T3" -- -d $D -c samplerate=10m:captureratio=100 --samples 40 -t 6=f -O bits
    emu_case "trigger 1 MHz" --digest --filter "$T" -- -d $D -c samplerate=1m:captureratio=100 --samples 40 -t 5=r -O bits
    emu_case "trigger 200 kHz" --digest --filter "$T" -- -d $D -c samplerate=200k:captureratio=100 --samples 40 -t 5=r -O bits
    # The 100/200 MHz netlists: one channel's edge.
    # Channel 6 rises at sample 64, falls at 128.
    emu_case "100 MHz rising" --digest --filter "$T2" -- -d $D -c samplerate=100m:$R --samples 40 -t 6=r -O bits
    emu_case "100 MHz falling" --digest --filter "$T3" -- -d $D -c samplerate=100m:$R --samples 40 -t 6=f -O bits
    emu_case "100 MHz level" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=100m --samples 40 -t 3=1
    emu_case "100 MHz two pins" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=100m --samples 40 -t 3=r,4=r
    # Four channels at 200 MHz: a slower count puts the edge past the
    # first 28 events.
    asix_sigma_options step=20
    emu_case "200 MHz rising" --digest --filter "$T3" -- -d $D -c samplerate=200m:captureratio=100 --samples 40 -t 4=r -O bits
    # On the time limit, as "trigger level".
    emu_case "200 MHz falling" --digest --filter "$T3" -- -d $D -c samplerate=200m --samples 40 -t 4=f -O bits

    # Run length compression: a count that changes every 100000
    # samples, clusters only at the changes and the timestamp wraps.
    asix_sigma_options step=100000
    # Recorded: one memory row, so the fixture stays small (a dense
    # capture's readout is 2 MB of hex).
    emu_case "sparse" --digest --filter "$LONG" -- -d $D -c samplerate=50m --samples 150000 -O hex
    emu_case "sparse 50 MHz" --record -- -d $D -c samplerate=50m --samples 1000 -O hex
    emu_case "sparse 200 MHz" --record -- -d $D -c samplerate=200m --samples 100 -O hex
    emu_case "sparse trigger" --digest --filter "$LONG" -- -d $D -c samplerate=50m --samples 150000 -t 2=r -O hex

    # The variants.
    asix_sigma_options variant=sigma2
    # Attached again (other strings): an address the READY line did not report.
    emu_case "sigma2 scan" --keep-header --filter 's/conn=[0-9]+[.][0-9]+/conn=BUS.ADDR/' -- -d $D --scan
    emu_case "sigma2 capture" -- -d $D -c samplerate=50m --samples 20 -O hex
    # Another product id of ASIX's: not a SIGMA.
    asix_sigma_options product=0xa001
    emu_case "other product" --no-output -- -d $D --scan
    asix_sigma_options variant=omega
    emu_case "omega scan" --stderr --filter "$E" -- -d $D --scan
    asix_sigma_options serial=
    emu_case "no serial" --stderr --filter "$E" -- -d $D --scan
    asix_sigma_options serial=A601XYZ
    emu_case "bad serial" --stderr --filter "$E" -- -d $D --scan
    asix_sigma_options serial=A6040001
    emu_case "unknown serial" --stderr --filter "$E" -- -d $D --scan

    # Faults of the board.
    asix_sigma_options init-busy=2
    emu_case "init busy" -- -d $D -c samplerate=50m --samples 20 -O hex
    asix_sigma_options config=fail
    emu_case "configuration fails" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options id=0x55
    emu_case "wrong id" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options scratch=stuck
    emu_case "scratch stuck" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options scratch=bit0
    emu_case "scratch bit stuck" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options stop-polls=3
    emu_case "slow stop" -- -d $D -c samplerate=50m --samples 20 -O hex
    # A forced stop that takes longer than oxrok waits: sigrok-cli's poll
    # has no limit.
    asix_sigma_options stop-polls=1200
    emu_case "stop never seen" --oracle-differs "protocol.c:1995-2001 poll the mode register after a forced stop for ever" --stderr --filter "$E" -- -d $D -c samplerate=50m --samples 20 -O hex
    # A late stop register in the middle of a cluster (the model's doc):
    # with no sample limit to hide it, both tools decode up to the
    # decremented position, and stay out of what was not written.
    asix_sigma_options stop-mid=6
    emu_case "stop register late, cluster unwritten" --digest --filter "$LONG" -- -d $D -c samplerate=50m --time 2 -O hex
    asix_sigma_options stop-mid=3
    emu_case "stop register late, mid cluster" --digest --filter "$LONG" -- -d $D -c samplerate=50m --time 2 -O hex
    # The FTDI shell's faults. A read of the pins with nothing sampled
    # yet: the driver sleeps and reads again; never anything: it gives
    # up after 10 times 10 reads.
    asix_sigma_options idle=1
    emu_case "idle read" -- -d $D -c samplerate=50m --samples 20 -O hex
    asix_sigma_options idle=1000
    emu_case "init never seen" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options stall=reset/0
    emu_case "open fails" --stderr --expect-fail --filter "$E" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options stall=bitmode/0x01df
    emu_case "bit-bang fails" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options stall=baud/0
    emu_case "bit rate fails" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    asix_sigma_options stall=bitmode/0
    emu_case "leaving bit-bang fails" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20
    # The purges' results are ignored.
    asix_sigma_options stall=reset/1
    emu_case "purge fails" -- -d $D -c samplerate=50m --samples 20 -O hex
    # The bulk IN endpoint halts after N transfers: the pins of the
    # upload (0), the drain after it (1), the netlist's answer (2), then
    # the capture's polls and the readout.
    for n in 0 1 2; do
        asix_sigma_options in-stall=$n
        emu_case "read fails after $n" --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20 -O hex
    done
    for n in 3 4 5 6 7; do
        asix_sigma_options in-stall=$n
        emu_case "read fails after $n" --stderr --filter "$E" -- -d $D -c samplerate=50m --samples 20 -O hex
    done
    asix_sigma_options
    # Unplugged after the Nth bulk OUT transfer: the bulk write that
    # follows fails. 1 a suicide burst, 4 before the PROG pulse, 6 the
    # second bitstream chunk, 15 trigger programming entered, before the
    # first table word, 16 before its strobe, 47 before the trigger
    # parameters, 48 the trigger options, 49 leaving programming, 50
    # the clock, 51 the post trigger, 52 the start; those fail the
    # start. 53 the first poll's commands, 60 the row fetches of the
    # readout: the acquisition ends with an error.
    for n in 1 4 6 15 16 47 48 49 50 51 52; do
        emu_case "unplug after write $n" --unplug out=2:$n --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20 -O hex
    done
    for n in 53 60; do
        emu_case "unplug after write $n" --unplug out=2:$n --stderr --filter "$E" -- -d $D -c samplerate=50m --samples 20 -O hex
    done
    # Unplugged after the Nth bulk IN transfer, where a write follows a
    # read: 3 the netlist's answer (trigger programming fails), 4 the
    # first poll (the mode read's command), 5 the mode read of the
    # readout (the forced stop), 6 the stop poll (read enable), 7 the
    # positions (the memory row).
    emu_case "unplug after read 3" --unplug in=1:3 --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=50m --samples 20 -O hex
    for n in 4 5 6 7; do
        emu_case "unplug after read $n" --unplug in=1:$n --stderr --filter "$E" -- -d $D -c samplerate=50m --samples 20 -O hex
    done
    # At 100 MHz (one more bitstream transfer): the 0x81 after entering
    # trigger programming.
    emu_case "unplug 100 MHz after write 16" --unplug out=2:16 --stderr --expect-fail --filter "$F" -- -d $D -c samplerate=100m --samples 20 -O hex
    emu_case "unplug at the first poll" --unplug model --stderr --filter "$E" -- -d $D -c samplerate=50m --samples 20 -O hex
}
