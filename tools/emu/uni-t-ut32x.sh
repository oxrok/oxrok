# A UNI-T UT32x thermometer on a plain serial line (emu/src/models/uni-t-ut32x.rs),
# on the CDC-ACM shell. Its own cable is tools/emu/uni-t-ut32x-hid.sh.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=uni-t-ut32x
EMU_CDC=1
EMU_DRIVER=uni-t-ut32x
EMU_FULL=$EMU_DRIVER
EMU_GCOV=uni-t-ut32x
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_conns() {
    A=uni-t-ut32x:conn=$EMU_TTY
}

# A record as hex: type, four characters of temperature, unit, slot, probe. The
# rest is the wiki's layout.
ut32x_rec() {
    emu_hex "$1$2$3${4}01230$5"'001\r\n'
}

# ut32x_list NAME LIMIT RECORD...: a capture of LIMIT samples on these records,
# one every 2 offers and then silence, so that no count depends on when a stop
# lands.
ut32x_list() {
    ut32x_name=$1
    ut32x_limit=$2
    shift 2
    ut32x_all=
    ut32x_count=0
    for ut32x_one in "$@"; do
        ut32x_all=$ut32x_all$ut32x_one,
        ut32x_count=$((ut32x_count + 1))
    done
    emu_options period=2 burst=$ut32x_count live=${ut32x_all%,}
    emu_case "$ut32x_name" -- -d $A --samples $ut32x_limit
}

emu_cases() {
    emu_conns
    # oxrok's log prefix, as C's.
    F='s/^oxrok: [a-z]+: oxrok::drivers::uni_t_ut32x(::protocol)?: /sr: uni-t-ut32x: /'
    # What oxrok adds after the error sigrok-cli names.
    FN="$F
s/^(Failed to set device option .[a-z_]+.: invalid argument).*$/\\1./"
    CHAN_BUG="process_packet (protocol.c:125-142) sends what the meter shows whatever -C says; oxrok sends a channel that is on, and counts the rest as C does"
    DASH="both tools read the dash of T1-T2 as a range and refuse it, in their own words, so that channel cannot be named alone"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture and sends the stop"
    UNTIL_PACKET="protocol.c:164-167 checks the limits only behind a packet, so a device that says nothing never ends a capture; oxrok ends it"
    UNPLUG_BUG="ut32x_handle_events (protocol.c:296-297) reads on G_IO_IN alone, and a gone tty polls as G_IO_HUP, so it goes on for ever; oxrok gets an error and ends"
    SPLIT_BUG="ut32x_receive_data (protocol.c:269-272) drops a full buffer ending in CR, so the LF starts the next one and every record after it is cut the same way, for ever; oxrok drops that LF too and is in step at the next record"
    T1=$(ut32x_rec 2 :235 1 00 0)
    T2=$(ut32x_rec 2 :241 1 00 1)

    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "no conn" --keep-header --no-output -- -d uni-t-ut32x --scan
    emu_case "no such node" --keep-header --no-output -- -d uni-t-ut32x:conn=/dev/ttyACM97 --scan
    emu_case "show without a device" --keep-header --stderr --filter "$F" --expect-fail -- -d uni-t-ut32x --show
    # C opens the tty and then fails to set the line; oxrok fails before it opens.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $A:serialcomm=fast --scan
    emu_case "serialcomm another rate is no matter to a scan" --keep-header -- -d $A:serialcomm=9600/8n1 --scan

    emu_case "get limit_samples" -- -d $A --get limit_samples
    emu_case "get limit_time" -- -d $A --get limit_time
    emu_case "get data_source" -- -d $A --get data_source
    emu_case "set then get limit_samples" -- -d $A -c limit_samples=7 --get limit_samples
    emu_case "set then get limit_time" -- -d $A -c limit_time=70 --get limit_time
    emu_case "set then get data_source" -- -d $A -c data_source=Memory --get data_source
    emu_case "set data_source wrong" --stderr --filter "$FN" --expect-fail -- -d $A -c data_source=memory --set
    emu_case "get what it has not" --stderr --filter "$F" --expect-fail -- -d $A --get samplerate

    emu_options period=2 burst=1
    emu_case "one sample" -- -d $A --samples 1
    emu_options period=2 burst=3
    emu_case "three samples" -- -d $A --samples 3
    emu_case "three samples analog" -- -d $A --samples 3 -O analog
    emu_case "three samples csv" -- -d $A --samples 3 -O csv:header=false:time=false
    emu_case "config limit_samples" --libsigrok --oxrok -- -d $A -c limit_samples=3 --continuous
    emu_case "frames" --stderr --filter "$F" --expect-fail -- -d $A --frames 1
    emu_case "one channel on" --oracle-differs "$CHAN_BUG" -- -d $A -C T2 --samples 3
    emu_case "the channel with a dash" --oracle-bug "$DASH" --stderr --filter "$F" --expect-fail -- -d $A -C T1-T2 --samples 3
    emu_options period=2 burst=3 chunk=1
    emu_case "one byte per transfer" -- -d $A --samples 3
    emu_options period=2 burst=3 uart=9600/8n1
    emu_case "another line, as the device is set" -- -d $A:serialcomm=9600/8n1 --samples 3

    # The values, all of which count.
    ut32x_list "celsius, fahrenheit, kelvin" 3 $T1 $(ut32x_rec 2 :235 2 00 0) $(ut32x_rec 2 :235 3 00 0)
    ut32x_list "unit zero, as in recall" 1 $(ut32x_rec 0 :235 0 00 0)
    ut32x_list "unknown unit" 2 $(ut32x_rec 2 :235 9 00 0) $T1
    ut32x_list "the probes" 4 $T1 $T2 $(ut32x_rec 2 :012 1 00 2) $(ut32x_rec 2 :012 1 00 3)
    ut32x_list "negative" 2 $(ut32x_rec 2 ';:12' 1 00 0) $(ut32x_rec 2 ';123' 1 00 0)
    ut32x_list "negative zero" 1 $(ut32x_rec 2 ';:::' 1 00 0)
    ut32x_list "all blank" 1 $(ut32x_rec 2 :::: 1 00 0)
    ut32x_list "the most digits" 2 $(ut32x_rec 2 9999 1 00 0) $(ut32x_rec 2 0001 1 00 0)
    ut32x_list "a sign anywhere" 2 $(ut32x_rec 2 '12;3' 1 00 0) $(ut32x_rec 2 '123;' 1 00 0)
    # Counted with no sample: the limit comes on the good one.
    ut32x_list "no probe counts" 2 $(ut32x_rec 2 ';;;;' 1 00 0) $T1
    ut32x_list "two signs count" 2 $(ut32x_rec 2 ';;12' 1 00 0) $T1
    ut32x_list "a wrong digit counts" 2 $(ut32x_rec 2 '1x34' 1 00 0) $T1
    ut32x_list "an unknown probe counts" 2 $(ut32x_rec 2 :235 1 00 7) $T1
    # Whatever the type digit, the slot, the time and the unknown two.
    ut32x_list "the fields it does not read" 2 $(emu_hex '7:235199012590ab1\r\n') $T1

    # Not counted: the limit of 1 comes on the good one behind them.
    ut32x_list "byte 8 wrong" 1 $(emu_hex '2:235100x12300001\r\n') $T1
    ut32x_list "byte 16 wrong" 1 $(emu_hex '2:23510001230000x\r\n') $T1
    ut32x_list "short packet" 1 $(emu_hex 'short\r\n') $T1
    # Noise of exactly the buffer: dropped whole, and the record behind it is good.
    ut32x_list "noise of a whole buffer" 1 $(emu_hex 'xxxxxxxxxxxxxxxxxxx') $T1
    # Five bytes ahead of a record: cut at 19, and its tail is a short packet.
    ut32x_list "a record cut by noise is lost" 1 $(emu_hex 'xxxxx') $T1 $T2
    ut32x_list "two bytes ahead of a record" 1 $(emu_hex '\n\n') $T1 $T2
    ut32x_list "a marker in the noise" 1 $(emu_hex 'xx\r\nxx') $(emu_hex '\r\n') $T1
    ut32x_list "markers alone" 1 $(emu_hex '\r\n\r\n\r\n') $T1
    ut32x_list "two records in one go" 2 $T1$T2
    # One byte ahead of a record puts the record's CR at the buffer's end, and C stays
    # out of step for as long as nothing else shifts it: so one such byte, and only
    # records behind it. The period leaves oxrok's stop ahead of the next record.
    ut32x_after=$T1,$T2,$T1,$T2,$T1,$T2,$T1,$T2
    emu_options period=10 burst=9 live=$(emu_hex 'x'),$ut32x_after
    emu_case "one byte ahead of a record" --oracle-bug "$SPLIT_BUG" --oracle-hangs --timeout 6 -- -d $A --samples 1
    emu_options period=10 burst=9 live=$(emu_hex 'xxxxxxxxxxxxxxxxxx\r\n'),$ut32x_after
    emu_case "a long packet" --oracle-bug "$SPLIT_BUG" --oracle-hangs --timeout 6 -- -d $A --samples 1

    # The memory.
    emu_options period=1 slots=5
    emu_case "memory" -- -d $A -c data_source=Memory --samples 5
    emu_options period=1 burst=3
    emu_case "memory, three of its slots" -- -d $A -c data_source=Memory --samples 3
    emu_options period=1 slots=5 stored=$(ut32x_rec 0 :198 1 00 0),$(ut32x_rec 0 ';;;;' 0 01 0),$(ut32x_rec 0 :201 2 02 1)
    emu_case "memory with a hole" -- -d $A -c data_source=Memory --samples 3

    # A time limit bites behind a packet. A period of 10 offers, some 200 ms: the
    # first record comes well past 50 ms, and it ends the capture with that one.
    emu_options period=10
    emu_case "time" -- -d $A --time 50
    emu_case "time and samples, the time first" -- -d $A --time 50 --samples 5
    emu_options period=2 burst=2
    emu_case "time and samples, the samples first" -- -d $A --time 60000 --samples 2

    # Silence.
    emu_options burst=0
    emu_case "time, silent" --oracle-bug "$UNTIL_PACKET" --oracle-hangs --timeout 5 --no-output -- -d $A --time 50
    emu_options period=2 burst=1 uart=9600/8n1
    emu_case "time, the line wrong" --oracle-bug "$UNTIL_PACKET" --oracle-hangs --timeout 5 --no-output -- -d $A --time 50
    # Over 10 s: the bound oxrok waits, and C's timeout beyond it. One case per form.
    emu_options burst=0
    emu_case "samples, silent says so" --oracle-bug "$UNTIL_PACKET" --oracle-hangs --timeout 8 --stderr --filter "$F" -- -d $A --samples 1

    # A stop, from a client, and from a signal. A long period: the client's stop lands
    # ahead of the second record.
    emu_options period=10 burst=2
    emu_case "lib stop" --libsigrok -- -d $A --continuous --stop-after 1
    emu_options period=2 burst=2
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $A --samples 5
    emu_case "unplug" --oracle-bug "$UNPLUG_BUG" --oracle-hangs --unplug model --timeout 4 -- -d $A --samples 5
    emu_case "unplug says so" --oracle-bug "$UNPLUG_BUG" --oracle-hangs --unplug model --timeout 4 --stderr --filter "$F" -- -d $A --samples 5
    emu_options
}
