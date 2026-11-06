# A Fluke 45 multimeter (emu/src/models/fluke-45.rs) on the CDC-ACM shell. Sourced
# by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=fluke-45
EMU_CDC=1
EMU_DRIVER=fluke-45
EMU_FULL=$EMU_DRIVER
EMU_GCOV=fluke-45
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_conns() {
    K=fluke-45:conn=$EMU_TTY
}

emu_cases() {
    emu_conns
    # Where the prompts fall against the commands is the one thing that differs in
    # every capture: C sends its next command on the answer's line end and finds the
    # prompt ahead of the next answer, and at its stop reads one more; oxrok waits for
    # the prompt, as the manual says. Taken out of each direction's bytes here, with
    # the lengths that change with them: "prompt order" pins the order.
    P='s/^bulk_in 82 [0-9]+ -> /bulk_in 82 -> /
s/3d3e0d0a//g
/^bulk_in 82 -> $/d'
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: (err|warn): oxrok::drivers::fluke_45(::protocol)?: /sr: fluke-45: /'
    fc() { fc_name=$1; shift; emu_case "$fc_name" --tx-filter "$P" "$@"; }
    # The same, cut where the clock takes over; a later --tx-filter replaces an earlier.
    upto() { printf '%s\n%s' "$P" "$1"; }
    MOD='/^bulk_out 02 4d4f443f0a/q'
    FUNC1='/^bulk_out 02 46554e43313f0a/q'

    WAIT="protocol.c:320-351 sends the next command on the last one's answer, ahead of its prompt, where the manual says a host must wait for it, and api.c:209 reads one line more at a stop; oxrok reads each command's prompt before the next"
    CRASH="protocol.c:265 strcmp()s the answer with no NULL check, which is what a failed or refused VAL1? leaves: a crash; oxrok has no sample for that round and goes on"
    SILENT="a silent meter is asked for ever and nothing in C ends a capture on a sample limit; oxrok gives up"
    TOGETHER="scpi_serial.c takes a read that ends in LF as one answer, so an answer and its prompt in one read make 'VDC\\r\\n=>', no function, and no reading ever; oxrok reads lines"
    SIGINT="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    CSV="csv.c:191 counts the channels a device has and :713 prints a row when that many packets came, so with P2 disabled it loses every other reading; oxrok prints them all"
    EARLY="under the manual's word that a string ahead of the prompt is discarded, C's every second command is lost"

    # A scan: one question, and one more after a reopen if it gets nothing.
    emu_options
    fc "scan" --keep-header -- -d $K --scan
    fc "serialcomm" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    fc "serialcomm of another line" --keep-header -- -d $K:serialcomm=2400/7e1 --scan
    emu_case "no such node" --keep-header --no-output -- -d fluke-45:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d fluke-45 --scan
    fc "show" --keep-header -- -d $K --show
    for key in limit_samples limit_time conn continuous; do
        fc "get $key" --expect-fail --stderr -- -d $K --get $key
    done
    fc "set limit_samples" --no-output -- -d $K -c limit_samples=2 --set
    fc "set limit_time" --no-output -- -d $K -c limit_time=100 --set
    emu_options idn=fluke,45,123,1.6
    fc "maker in lower case" --keep-header -- -d $K --scan
    emu_options idn=FLUKE,45,1.6
    fc "short id" --keep-header -- -d $K --scan
    emu_options idn=FLUKE,8846A,1,2
    fc "another model" --keep-header --no-output -- -d $K --scan
    emu_options idn=ACME,45,1,2
    fc "another maker" --keep-header --no-output -- -d $K --scan
    emu_options idn=FLUKE,45
    fc "two fields" --keep-header --no-output -- -d $K --scan
    emu_options silent-at=1
    fc "scan, asked again" --keep-header -- -d $K --scan
    emu_options mute-at=1
    fc "scan of a silent meter" --keep-header --no-output -- -d $K --scan
    emu_options "say=1:=>"
    fc "a prompt for an id" --keep-header -- -d $K --scan

    # A capture: a round is a sample, whatever it holds.
    emu_options
    emu_case "prompt order" --oracle-differs "$WAIT" -- -d $K --samples 1
    fc "samples 1" -- -d $K --samples 1
    fc "samples 3 analog" -- -d $K --samples 3 -O analog
    fc "samples csv" --oracle-differs "$CSV" -- -d $K -C P1 --samples 2 -O csv:header=false:time=false
    fc "config limit_samples" -- -d $K -c limit_samples=2 --samples 1
    fc "channel P1" -- -d $K -C P1 --samples 1
    fc "no such channel" --stderr --filter 's/^Failed to set channels: invalid argument: //' --expect-fail -- -d $K -C P3 --samples 1
    # Time decides how many: kept up to the first.
    fc "time" --filter '2,$d' --tx-filter "$(upto "$MOD")" -- -d $K --time 1500ms
    fc "control lines stalled" -- -d $K --samples 1

    # The label of each function, and of each modifier on the first display.
    for f in AAC ADC CONT DIODE FREQ OHMS VAC VACDC; do
        emu_options func=$f
        fc "function $f" -- -d $K --samples 1 -O analog
    done
    emu_options func=AACDC
    fc "function AACDC" --oracle-differs "protocol.c:68-71 gives AACDC AC alone, where VACDC at :93-97 has AC and DC; oxrok has both" -- -d $K --samples 1 -O analog
    emu_options auto=0
    fc "no autorange" -- -d $K --samples 1 -O analog
    for m in 1 2 4 8 16 64 3 9; do
        emu_options mod=$m
        fc "modifiers $m" -- -d $K --samples 1 -O analog
    done
    REL="protocol.c:162-165 says HOLD of REL; oxrok says relative"
    emu_options mod=32
    fc "modifiers REL" --oracle-differs "$REL" -- -d $K --samples 1 -O analog
    emu_options mod=40
    fc "modifiers dB and REL" --oracle-differs "$REL" -- -d $K --samples 1 -O analog

    # The readings, across classes of value.
    emu_options values=+1E+9
    fc "overload" -- -d $K --samples 1 -O analog
    emu_options values=-1E+9
    fc "negative of an overload" -- -d $K --samples 1 -O analog
    emu_options values=+0.0000E+0,-9.9999E+2,+1.2345E-3,+3.000E+8,+5E-1,1.5E+0,+1.5E+10
    fc "values" -- -d $K --samples 7 -O analog

    # Two displays.
    emu_options func2=FREQ
    fc "two displays" -- -d $K --samples 2 -O analog
    fc "two displays, csv" -- -d $K --samples 2 -O csv:header=false:time=false
    fc "channel P2" -- -d $K -C P2 --samples 1
    fc "channel P1 of two" -- -d $K -C P1 --samples 1
    emu_options func=FREQ func2=DIODE
    fc "two displays, a diode on the second" -- -d $K --samples 1 -O analog
    emu_options func2=CONT
    # protocol.c:78 sets analog[0]'s unit, and the output of neither shows it.
    fc "continuity on the second" -- -d $K --samples 1 -O analog
    emu_options func2=FREQ mod=8
    fc "dB on both" -- -d $K --samples 1 -O analog

    # What the meter says it can't, or says that isn't what was asked. Counted from
    # 1 with the scan's *IDN?, so the round is FUNC1? 2, AUTO? 3, VAL1? 4, MOD? 5,
    # FUNC2? 6, AUTO? 7.
    emu_options "say=2:!>"
    fc "FUNC1 refused" -- -d $K --samples 1
    emu_options "say=2:?>"
    fc "FUNC1 a command error" -- -d $K --samples 1
    emu_options "say=3:!>"
    fc "AUTO refused" -- -d $K --samples 1 -O analog
    emu_options "say=5:!>"
    fc "MOD refused" -- -d $K --samples 1 -O analog
    emu_options "say=4:!>"
    fc "VAL1 refused" --oracle-bug "$CRASH" -- -d $K --samples 1
    emu_options func2=FREQ "say=8:!>"
    fc "VAL2 refused" --oracle-bug "$CRASH" -- -d $K --samples 1
    emu_options say=2:BOGUS
    fc "FUNC1 a word it has not" -- -d $K --samples 1
    emu_options say=3:2
    fc "AUTO neither" -- -d $K --samples 1 -O analog
    emu_options say=5:abc
    fc "MOD no number" -- -d $K --samples 1 -O analog
    emu_options say=5:40abc
    fc "MOD part a number" --oracle-differs "protocol.c:133 atoi() takes 40 of 40abc, for dBm and hold on a reading; oxrok has no such modifiers" -- -d $K --samples 1 -O analog

    # Lines cut, slow, or the prompt elsewhere.
    # A gap long enough for a line's bytes, one per transfer, to be out: when they
    # meet, sr_atof_ascii fails and C prints what its stack holds.
    emu_options chunk=1 line-gap=40
    fc "one byte per transfer" -- -d $K --samples 2
    emu_options line-gap=12
    fc "a slower line" -- -d $K --samples 2
    fc "a time shorter than a round" -- -d $K --time 1ms
    emu_options prompts=none
    fc "no prompts" -- -d $K --samples 2
    emu_options prompts=together
    fc "prompt in one read" --oracle-bug "$TOGETHER" --oracle-hangs --timeout 4 -- -d $K --samples 2
    fc "prompt in one read, a time limit" --oracle-differs "$TOGETHER" --filter '2,$d' --tx-filter "$(upto "$MOD")" -- -d $K --time 1500ms
    emu_options early=discard
    fc "the manual's word on a command ahead of the prompt" --oracle-bug "$EARLY" --oracle-hangs --timeout 4 -- -d $K --samples 2

    # Late and silent answers.
    emu_options late-at=4:10
    fc "a reading a little late" -- -d $K --samples 1
    emu_options late-at=4:100
    fc "a reading too late for C" --oracle-bug "$CRASH" -- -d $K --samples 2
    emu_options silent-at=4
    fc "a reading lost" --oracle-bug "$CRASH" -- -d $K --samples 1
    emu_options mute-at=4
    fc "readings lost for good, a time limit" --oracle-bug "$CRASH" --stderr --filter "$L" -- -d $K --time 1500ms
    emu_options mute-at=2
    fc "silent, a time limit" --no-output --tx-filter "$(upto "$FUNC1")" -- -d $K --time 1500ms
    fc "silent, a time limit, told" --oracle-differs "C says nothing of a capture with nothing to show" --stderr --filter "$L" --tx-filter "$(upto "$FUNC1")" -- -d $K --time 1500ms
    # The only real-binary run of the wait for the first sample: 30 s, as a manual's
    # slow rate and a settling range have room in it.
    fc "silent, a sample limit" --oxrok-fails "no answer from the meter for 30 s" --oracle-bug "$SILENT" --oracle-hangs --no-output --timeout 34 --tx-filter "$(upto "$FUNC1")" -- -d $K --samples 1

    # An unplug and a stop.
    emu_options mark-at=8
    # sigrok-cli does not end: it asks again on each failed write, with a line on
    # stderr for each, over 400000 of them in 4 s and never the same number, too many
    # to hold as an --oracle-bug golden. Its result is evidence from runs. Written by
    # hand, from a blessed run as an --oracle-bug case.
    fc "unplug" --no-oracle-only --oxrok-fails "error" --unplug model --timeout 8 --tx-filter "$(upto '/^unplug$/q')" -- -d $K --samples 5
    emu_options mark-at=8
    # A stalled tool takes its signal late, and has a reading more by then.
    fc "interrupt" --oracle-bug "$SIGINT" --filter '2,$d' --interrupt model --timeout 8 --tx-filter "$(upto '/^interrupt$/q')" -- -d $K --continuous
}
