# A B&K Precision 1856D frequency counter (emu/src/models/bkprecision-1856d.rs)
# on the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=bkprecision-1856d
EMU_CDC=1
EMU_DRIVER=bkprecision-1856d
EMU_GCOV=bkprecision-1856d
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # A frame of the 15 bytes the parser reads, as the model's replies= wants it.
    R() { emu_hex "$1"; }

    # A scan does not open the port and takes any name (api.c's scan).
    emu_options
    emu_case "scan" --keep-header -- -d $B --scan
    emu_case "scan no conn" --keep-header --no-output -- -d bkprecision-1856d --scan
    emu_case "scan no such node" --keep-header -- -d bkprecision-1856d:conn=/dev/ttyACM97 --scan
    # serialcomm is not in scanopts, and a scan with it finds nothing.
    emu_case "scan serialcomm is not its option" --keep-header --no-output -- -d $B:serialcomm=2400/8n1 --scan

    # --show and --get: what the settings say with no capture, and nothing on the wire.
    emu_case "show" --keep-header -- -d $B --show
    emu_case "show no such node" --stderr --expect-fail --keep-header -- -d bkprecision-1856d:conn=/dev/ttyACM97 --show
    emu_case "get gate_time" -- -d $B --get gate_time
    emu_case "get data_source" -- -d $B --get data_source
    emu_case "get limit_samples is not gettable" --stderr --expect-fail -- -d $B --get limit_samples
    emu_case "get conn is not gettable" --stderr --expect-fail -- -d $B --get conn
    emu_case "get continuous is not gettable" --stderr --expect-fail -- -d $B --get continuous
    for g in 10ms 100ms 1s 10s; do
        emu_case "set gate_time $g reads back" -- -d $B -c gate_time=$g --get gate_time
    done
    emu_case "show gate_time 10s" --keep-header -- -d $B -c gate_time=10s --show
    emu_case "set gate_time not in the list" --stderr --expect-fail -- -d $B -c gate_time=2s --get gate_time
    emu_case "set gate_time 1000ms" --stderr --expect-fail -- -d $B -c gate_time=1000ms --get gate_time
    emu_case "set gate_time 0.1s" --stderr --expect-fail -- -d $B -c gate_time=0.1s --get gate_time
    emu_case "set gate_time no number" --stderr --expect-fail -- -d $B -c gate_time=abc --get gate_time
    for s in A C; do
        emu_case "set data_source $s reads back" -- -d $B -c data_source=$s --get data_source
    done
    emu_case "show data_source A" --keep-header -- -d $B -c data_source=A --show
    emu_case "set data_source not in the list" --stderr --expect-fail -- -d $B -c data_source=B --get data_source
    emu_case "set data_source a small letter" --stderr --expect-fail -- -d $B -c data_source=a --get data_source
    emu_case "set no such key" --stderr --expect-fail -- -d $B -c volage=1 --get gate_time
    emu_case "set limit_samples" --no-output -- -d $B -c limit_samples=5 --set
    emu_case "set limit_samples 0" --stderr --expect-fail -- -d $B -c limit_samples=0 --set
    emu_case "set continuous has no set" --stderr --expect-fail -- -d $B -c continuous=on --set

    # A capture: F, G, D, and for each reading G and D again.
    emu_case "samples 3" -- -d $B --samples 3
    emu_case "samples 1" -- -d $B --samples 1
    emu_case "samples through -c" -- -d $B -c limit_samples=2 --samples 2
    emu_case "samples 0" --stderr --expect-fail -- -d $B --samples 0
    emu_case "samples no number" --stderr --expect-fail -- -d $B --samples abc
    emu_case "time is not offered" --stderr --expect-fail -- -d $B --time 100ms
    emu_case "frames is not offered" --stderr --expect-fail -- -d $B --frames 2
    emu_case "no such channel" --stderr --expect-fail -- -d $B -C Z --samples 1
    emu_case "no such node" --stderr --expect-fail -- -d bkprecision-1856d:conn=/dev/ttyACM97 --samples 1

    # What -c changes on the wire, and in the reading.
    for g in 100ms 1s 10s; do
        emu_case "capture gate $g" -- -d $B -c gate_time=$g --samples 1
    done
    # Each reading sends the gate again, and the sleep after G3 is 800 ms.
    emu_case "capture gate 10s twice" -- -d $B -c gate_time=10s --samples 2
    emu_case "capture input A" -- -d $B -c data_source=A --samples 2
    emu_case "capture input C" -- -d $B -c data_source=C --samples 2
    emu_case "capture input A gate 1s" -- -d $B -c data_source=A:gate_time=1s --samples 1

    # Readings across the ranges, each a line of the output, and how each prefix
    # counts in the digits.
    for hz in 1 999 1000 123456 1000000 12345678 100000000 3700000000; do
        emu_options c=$hz
        emu_case "reading of $hz Hz" -- -d $B --samples 1
    done
    emu_options c=12345678
    emu_case "reading analog" -- -d $B --samples 1 -O analog
    emu_case "reading csv" -- -d $B --samples 1 -O csv:header=false:time=false

    # The empty message, on a counter that has nothing yet, or nothing at all.
    emu_options settle=2
    emu_case "empty messages after each G" -- -d $B --samples 2
    emu_options c=0
    emu_case "lib no signal never ends" --libsigrok --hangs --timeout 2 --no-output --tx-filter "$NO_SIGNAL" -- -d $B --samples 1

    # The forms of a frame: what strtod, the digits, the prefix and the checks do
    # with each. All are 15 bytes.
    emu_options replies=$(R '   1.00000MHz \r')
    emu_case "frame MHz" -- -d $B --samples 1
    emu_options replies=$(R '   1.00000kHz \r')
    emu_case "frame kHz" -- -d $B --samples 1
    emu_options replies=$(R '   1.00000 Hz \r')
    emu_case "frame Hz" -- -d $B --samples 1
    emu_options replies=$(R '   1.00000GHz \r')
    emu_case "frame GHz has no factor" -- -d $B --samples 1
    emu_options replies=$(R '  12345678 Hz \r')
    emu_case "frame no point Hz" -- -d $B --samples 1
    emu_options replies=$(R '      1234MHz \r')
    emu_case "frame no point MHz" -- -d $B --samples 1
    emu_options replies=$(R '1.23456789 Hz \r')
    emu_case "frame eight decimals" -- -d $B --samples 1
    emu_options replies=$(R '         . Hz \r')
    emu_case "frame only a point" -- -d $B --samples 1
    emu_options replies=$(R 'abc.def    Hz \r')
    emu_case "frame no number before a point" -- -d $B --samples 1
    emu_options replies=$(R 'abcdefgh   Hz \r')
    emu_case "frame no number no point" -- -d $B --samples 1
    emu_options replies=$(R '12 3.4     Hz \r')
    emu_case "frame a point past the number" -- -d $B --samples 1
    emu_options replies=$(R '  -1.50000 Hz \r')
    emu_case "frame negative" -- -d $B --samples 1
    emu_options replies=$(R '  +1.50000 Hz \r')
    emu_case "frame a plus sign" -- -d $B --samples 1
    emu_options replies=$(R '   1.5e+03 Hz \r')
    emu_case "frame an exponent" -- -d $B --samples 1
    emu_options replies=$(R '    1.5e+3kHz \r')
    emu_case "frame an exponent and kHz" -- -d $B --samples 1
    emu_options replies=$(R '     1e400 Hz \r')
    emu_case "frame above a double" -- -d $B --samples 1
    emu_options replies=$(R '     1e-99 Hz \r')
    emu_case "frame under a float" -- -d $B --samples 1
    emu_options replies=$(R '      1e39 Hz \r')
    emu_case "frame above a float" -- -d $B --samples 1
    emu_options replies=$(R '       inf Hz \r')
    emu_case "frame inf" -- -d $B --samples 1
    emu_options replies=$(R '  infinity Hz \r')
    emu_case "frame infinity" -- -d $B --samples 1
    emu_options replies=$(R '      -inf Hz \r')
    emu_case "frame minus inf" -- -d $B --samples 1
    emu_options replies=$(R '       nan Hz \r')
    emu_case "frame nan" -- -d $B --samples 1
    emu_options replies=$(R '      -nan Hz \r')
    emu_case "frame minus nan" -- -d $B --samples 1
    emu_options replies=$(R '    nan(1) Hz \r')
    emu_case "frame nan with a tail" -- -d $B --samples 1
    emu_options replies=$(R '      0x10 Hz \r')
    emu_case "frame hex" -- -d $B --samples 1
    emu_options replies=$(R '   0x1.8p1 Hz \r')
    emu_case "frame hex float" -- -d $B --samples 1
    emu_options replies=$(R '\000  1.5     Hz \r')
    emu_case "frame NUL first" -- -d $B --samples 1
    emu_options replies=$(R '1.5\000.25    Hz \r')
    emu_case "frame NUL ahead of a point" -- -d $B --samples 1
    emu_options replies=$(R '0.00000000 Hz \r')
    emu_case "frame a zero with its unit" -- -d $B --samples 1
    emu_options replies=$(R '0000000000 Hz \r')
    emu_case "frame zeros no point" -- -d $B --samples 1
    emu_options replies=$(R '\377\376\375        Hz \r')
    emu_case "frame high bytes" -- -d $B --samples 1
    emu_options replies=$(R '1.0\t2\n\v\f\r\t Hz \r')
    emu_case "frame control bytes" -- -d $B --samples 1
    emu_options replies=$(R '\t\n\v\f   1.5 Hz \r')
    emu_case "frame space of all kinds" -- -d $B --samples 1

    # The empty message, and what only looks like one.
    B1=$(R '               ' | sed 's/20$/0d/')
    emu_options replies=$B1,$(R '   1.00000 Hz \r')
    emu_case "empty all spaces" -- -d $B --samples 1
    emu_options replies=$(R '0             \r'),$(R '   1.00000 Hz \r')
    emu_case "empty a zero first" -- -d $B --samples 1
    emu_options replies=$(R '             0\r'),$(R '   1.00000 Hz \r')
    emu_case "empty a zero last" -- -d $B --samples 1
    emu_options replies=$(R '     0  0     \r'),$(R '   1.00000 Hz \r')
    emu_case "two zeros are no empty message" --stderr -- -d $B --samples 1
    emu_options replies=$(R '     \000        \r'),$(R '   1.00000 Hz \r')
    emu_case "a NUL is no empty message" --stderr -- -d $B --samples 1

    # What fails the checks, each followed by a good frame: its log, and what goes
    # out again.
    emu_options replies=$(R '   1.00000 Hz \n'),$(R '   1.00000 Hz \r')
    emu_case "no CR at the end" --stderr -- -d $B --samples 1
    for unit in ms-'us  ' tail-' Hz_' left-'Hz  ' none-'    '; do
        emu_options replies=$(R "   1.00000${unit#*-}\r"),$(R '   1.00000 Hz \r')
        emu_case "not a frequency ${unit%%-*}" --stderr -- -d $B --samples 1
    done
    emu_options replies=$(R '   1.00000 Hz \r'),$(R '   1.00000 s  \r'),$(R '   2.00000 Hz \r')
    emu_case "not a frequency, the wire" -- -d $B --samples 2
    emu_options replies=$(R '   1.00000 Hz\000\r'),$(R '   1.00000 Hz \r')
    emu_case "not a frequency a NUL in the unit" --stderr -- -d $B --samples 1
    # A frame in pieces, and two in one.
    emu_options chunk=1
    emu_case "one byte per transfer" -- -d $B --samples 2
    emu_options replies=$(R '   1.00000 Hz \r   2.00000 Hz \r')
    emu_case "two frames at once" -- -d $B --samples 2
    # A frame too short waits for the rest, and the next frame is out of step.
    G2=$(R '   2.00000 Hz \r')
    emu_options replies=$(R '   1.00000 Hz\r'),$G2,$G2,$G2,$G2
    emu_case "a frame too short" --tx-filter "$FOLD" --stderr -- -d $B --samples 1
    # A frame too long: its surplus goes in the flush if it has come.
    emu_options replies=$(R '   1.00000 Hz \r\r'),$(R '   2.00000 Hz \r')
    emu_case "a frame too long" --stderr -- -d $B --samples 2

    # A request with no answer: 100 ms, then G and D again. The count of those
    # goes by the clock.
    emu_options replies=,$(R '   1.00000 Hz \r')
    emu_case "one answer missing" --tx-filter "$FOLD" -- -d $B --samples 1
    emu_options replies=$(R '   1.00000 Hz \r'),,$(R '   2.00000 Hz \r')
    emu_case "a later answer missing" --tx-filter "$FOLD" -- -d $B --samples 2
    emu_options late=1:15
    emu_case "a late answer" --tx-filter "$FOLD" -- -d $B --samples 1
    emu_options late=2:15
    emu_case "a late answer later" --tx-filter "$FOLD" -- -d $B --samples 2
    # After an expiry C's timer has gone by when G3's sleep ends, so G3 and D0 go
    # again before the answer can have come. session.c:108-109 took the time, in
    # fd_source_check, and :134 reuses it. After a reading the check took no time.
    emu_options replies=,$(R '   1.00000 Hz \r')
    emu_case "lib an expiry at gate 10s" --libsigrok -- -d $B -c gate_time=10s --samples 1
    # A silent counter: C goes on for ever.
    emu_options answers=0
    emu_case "lib silent never ends" --libsigrok --hangs --timeout 2 --no-output --tx-filter "$FOLD" -- -d $B --samples 1
    emu_case "silent never ends" --hangs --timeout 2 --no-output --tx-filter "$FOLD" -- -d $B --samples 1

    # The stops sigrok-cli has: --continuous at stdin's end, at once, and none else.
    emu_options
    emu_case "continuous ends at stdin" --no-output --tx-filter "$CONTINUOUS" -- -d $B --continuous
    emu_case "lib stop" --libsigrok -- -d $B --continuous --stop-after 2
    emu_case "lib stop at the header" --libsigrok --tx-filter "$CONTINUOUS" --no-output -- -d $B --continuous --stop-after 0

    # The port going.
    emu_options answers=2
    emu_case "unplug in a capture" --hangs --timeout 3 --stderr --unplug model -- -d $B --samples 3

    # The USB side misbehaving, the shell's options.
    emu_options stall=line
    # The kernel takes the stall and the line stays at 9600.
    emu_case "line coding stalled" -- -d $B --samples 1
    emu_options stall=lines
    emu_case "control lines stalled" --stderr --expect-fail -- -d $B --samples 1
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    B=bkprecision-1856d:conn=$EMU_TTY
    # A run of G and D with no answer between, one .tx line whose length goes by
    # the clock.
    FOLD='s/^bulk_out 02 ((46[0-9]{2}0d)?47[0-9]{2}0d44300d)(47[0-9]{2}0d44300d)+ -> [0-9]+$/bulk_out 02 \1 AGAIN/'
    # D alone and what it gets, on a counter with nothing to show.
    NO_SIGNAL='/^bulk_in 82 15 -> 20202020202020202030202020200d$/d
/^bulk_out 02 44300d -> 3$/d'
    # What came before the stop.
    CONTINUOUS='/^bulk_in/,$d'
}
