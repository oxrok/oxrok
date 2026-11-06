# A Norma DM9x0 / Siemens B102x multimeter (emu/src/models/norma-dmm.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# serial model". C registers two drivers from one directory, norma-dmm and
# siemens-b102x, which differ in their vendor and model strings.
EMU_MODEL=norma-dmm
EMU_CDC=1
EMU_DRIVER="norma-dmm siemens-b102x"
EMU_FULL="norma-dmm siemens-b102x"
EMU_GCOV=norma-dmm
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# A status's digits, as the driver reads them: 0 keyboard, 1 switch, 2 value
# type, 3 range, 4 flags (4 invalid, 2 duplicate, 1 overload), 5 sign (4) and
# first digit, 6-9 digits, 10 mode, 11 device status, 12-13, 14-15, 16-17 and
# 18-19 flag pairs.
emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only, at any level. C's serial layer has a
    # line of its own for a failed write, which oxrok's port does not log.
    L='s/^oxrok: (err|warn|info): oxrok::drivers::norma_dmm(::protocol)?: /sr: norma-dmm: /
/^sr: serial-libsp: /d'
    # sr_strerror's words against core::config_set's reason.
    # Exactly those words of oxrok's, so another wording fails.
    A='s/^(Failed to set device option .limit_samples.: invalid argument): it cannot be 0\.$/\1./'
    HEX="protocol.c:67-74: xgittoint gives a to f as 0 to 5, so its modes 0xa and 0xb (:248-258) and its defaults (:164-166, :256-258) never run, though the wiki's table has a diode as DC,DIODE and A~ as AC,DC,RMS, which only those modes give; oxrok gives each digit its value"
    PAIR="protocol.c:282, :300 and :316 shift a pair's first digit by 8 from the first patch on, so no mask of 0x10 or above ever holds (:287, :304, :319-320 never ran): continuity always 0, always AUTO, no warning, whatever a meter sends; oxrok shifts by 4. No recording settles it; if that is wrong the damage is flags, a warning and continuity's 0 or 1, never a numeric value"
    CUT="protocol.c:97 cuts a line at 20 digits with no look at the rest, so what ran into another line passes as a reading neither held; oxrok takes a 21st hex digit as a bad line"
    KEEP="protocol.c:417-421 sets nothing back when a request expires, so the part of a line stays and the next reply adds to it; oxrok drops the part"
    FULL_BUF="protocol.c:393 stops reading at 255 bytes with no LF, and the buffer never empties, so no more of the meter is read; oxrok takes it as one bad line"
    CR="protocol.c:399 ends a line at LF alone, and C says nothing of a meter that sends CR as the wiki writes; oxrok warns once"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    D950=1834_065_V1.06,IF_V1.02
    # The wiki's: 24.91 V DC.
    S=03030024913100400040
    # A request, for a .tx cut at the first.
    Q1='/^bulk_out 02 5354415455533f0d0a/q'

    emu_options
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "scan siemens" --driver siemens-b102x --keep-header -- -d $Z --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=4800/8n1 --scan
    emu_case "no such node" --keep-header --no-output -- -d norma-dmm:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d norma-dmm --scan
    # sigrok-cli opens the port before it reads the parameters, oxrok refuses them
    # first: the open's lines are all that differs.
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
    # The line coding reaches the wire: this meter hears 9600 alone.
    emu_options uart=9600/8n1
    emu_case "serialcomm reaches the line" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_options
    # The table: each type on each driver.
    # The DM950 is the first case's, and the B1028 the second's.
    for dmtype in 1 2 3 4; do
        emu_options idn=1834_06${dmtype}_V1.04,IF_V1.02
        emu_case "scan type $dmtype" --keep-header -- -d $K --scan
        emu_case "scan siemens type $dmtype" --driver siemens-b102x --keep-header -- -d $Z --scan
    done
    for dmtype in 0 6; do
        emu_options idn=1834_06${dmtype}_V1.04,IF_V1.02
        emu_case "scan type $dmtype" --keep-header -- -d $K --scan
    done
    emu_options idn=1834_06x_V1.04,IF_V1.02
    emu_case "scan type not hex" --keep-header -- -d $K --scan
    emu_options idn=1834_06a_V1.04,IF_V1.02
    emu_case "scan type a" --keep-header -- -d $K --scan
    emu_options idn=1834_06b_V1.04,IF_V1.02
    emu_case "scan type b" --oracle-differs "$HEX" --keep-header -- -d $K --scan
    # The least the regular expression takes: the type's place is the F of IF.
    emu_options idn=1834_,IF_
    emu_case "scan the shortest id" --oracle-differs "$HEX" --keep-header -- -d $K --scan
    emu_options idn=1834_065,IF_V
    emu_case "scan with no version" --keep-header -- -d $K --scan
    emu_options idn=1834_065_V1.06,IF_V1.02_with_words_up_to_the_end_of_the_buffer_and_on
    emu_case "scan id too long" --keep-header -- -d $K --scan
    # A line that does not match, then the good one. CR LF makes two reads of it.
    emu_options "idn=junk;$D950"
    emu_case "scan junk first" --keep-header -- -d $K --scan
    emu_options "idn=;$D950"
    emu_case "scan empty line first" --keep-header -- -d $K --scan
    emu_options idn=1835_065_V1.06,IF_V1.02
    emu_case "scan another number" --keep-header --no-output -- -d $K --scan
    emu_options idn=1834_065_V1.06,XF_V1.02
    emu_case "scan with no IF" --keep-header --no-output -- -d $K --scan
    emu_options idn=1834_065_V1.06
    emu_case "scan with no comma" --keep-header --no-output -- -d $K --scan
    # With LF alone each answer is one line, so the fourth try reaches C's wait of
    # 5 s, the only way to it: under CR LF that try falls on an empty read.
    emu_options eol=lf idn=junk
    emu_case "scan junk with LF waits" --timeout 120 --keep-header --no-output -- -d $K --scan
    # The interface charging: 4 s of silence, which has no such wait.
    emu_options "idn=~;~;$D950"
    emu_case "scan silent at first" --keep-header -- -d $K --scan
    # No line end: the read's 2 s run out and it takes what came.
    emu_options "idn=$D950~"
    emu_case "scan with no line end" --keep-header -- -d $K --scan
    emu_options eol=cr
    emu_case "scan with CR" --keep-header -- -d $K --scan
    emu_options eol=lf
    emu_case "scan with LF" --keep-header -- -d $K --scan
    emu_options chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $K --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    emu_options in-stall=1
    emu_case "a stalled read" --keep-header -- -d $K --scan
    emu_options answers=0
    emu_case "unplug in the scan" --stderr --filter "$L" --unplug model --timeout 10 --tx-filter '/^unplug$/q' -- -d $K --scan

    # --show, and what the driver has no answer to: config_get is NULL.
    emu_options
    emu_case "show" --keep-header -- -d $K --show
    emu_case "show siemens" --driver siemens-b102x --keep-header -- -d $Z --show
    for key in limit_samples limit_time conn continuous samplerate; do
        emu_case "get $key" --expect-fail --stderr -- -d $K --get $key
    done
    emu_case "set limit_samples" --no-output -- -d $K -c limit_samples=2 --set
    emu_case "set limit_time" --no-output -- -d $K -c limit_time=100 --set
    emu_case "set limit_samples 0" --stderr --filter "$A" --expect-fail -- -d $K -c limit_samples=0 --set
    emu_case "set continuous" --stderr --filter "$L" --expect-fail -- -d $K -c continuous=1 --set
    emu_case "set samplerate" --stderr --filter "$L" --expect-fail -- -d $K -c samplerate=1 --set

    # A capture: the first STATUS? at the source's first 100 ms, each next one
    # when the last has its line.
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 3 analog" -- -d $K --samples 3 -O analog
    emu_case "samples siemens" --driver siemens-b102x -- -d $Z --samples 1
    emu_case "samples csv" -- -d $K --samples 2 -O csv:header=false:time=false
    emu_case "config limit_samples" -- -d $K -c limit_samples=2 --samples 1
    emu_case "channel P1" -- -d $K -C P1 --samples 1
    emu_case "no such channel" --stderr --filter 's/^Failed to set channels: invalid argument: //' --expect-fail -- -d $K -C P2 --samples 1
    # Time decides the requests: kept up to the first.
    emu_case "time" --filter '3,$d' --tx-filter "$Q1" -- -d $K --time 500ms
    emu_case "time shorter than the first pass" --no-output -- -d $K --time 1ms
    emu_options chunk=1
    emu_case "samples one byte per transfer" -- -d $K --samples 2
    emu_options eol=lf
    emu_case "samples with LF" -- -d $K --samples 2

    # The value types and their ranges, the last ones having no range of their own.
    emu_options "status=$(norma_list 00 05 10 17 20 26 30 39 40 41 50 59 60 69 70 79 80 90)"
    emu_case "value types and ranges" -- -d $K --samples 18 -O csv:header=false:time=false
    # Sign, first digit, and a full reading.
    emu_options "status=$(norma_st 0 3 0 0 0000);$(norma_st 0 3 0 0 0001);$(norma_st 0 3 0 4 0000);$(norma_st 0 3 0 4 0001);$(norma_st 0 3 0 1 9999);$(norma_st 0 3 0 5 9999);$(norma_st 0 3 0 2 9999);$(norma_st 0 3 0 3 9999);$(norma_st 0 3 0 9 9999)"
    emu_case "signs and digits" -- -d $K --samples 9 -O analog
    emu_options "status=$(norma_st 0 3 0 0 2491 01);$(norma_st 0 3 0 0 2491 11);$(norma_st 0 3 0 0 2491 21);$(norma_st 0 3 0 0 2491 31);$(norma_st 0 3 0 0 2491 41);$(norma_st 0 3 0 0 2491 51);$(norma_st 0 3 0 0 2491 61);$(norma_st 0 3 0 0 2491 71);$(norma_st 0 3 0 0 2491 81);$(norma_st 0 3 0 0 2491 91)"
    emu_case "modes on a DM950" -- -d $K --samples 10 -O analog
    # Types under 3 have no RMS.
    AC="$(norma_st 0 3 0 0 2491 21);$(norma_st 1 3 0 0 2491 91)"
    emu_options "idn=1834_062_V1.04,IF_V1.02" "status=$AC"
    emu_case "AC on a DM920 has no RMS" -- -d $K --samples 2 -O analog
    emu_options "idn=1834_063_V1.04,IF_V1.02" "status=$AC"
    emu_case "AC on a DM930 is RMS" -- -d $K --samples 2 -O analog
    # The flags in the second digit of a pair: nothing, REL, %, both, HOLD, MAX,
    # MIN, LIMIT, dB on volts and on ohms; and digits that mean nothing to a sample.
    emu_options "status=$(norma_st 0 3 0 0 2491 31 00000000);$(norma_st 0 3 0 0 2491 31 04000000);$(norma_st 0 3 0 0 2491 31 01000000);$(norma_st 0 3 0 0 2491 31 05000000);$(norma_st 0 3 0 0 2491 31 00080000);$(norma_st 0 3 0 0 2491 31 00020000);$(norma_st 0 3 0 0 2491 31 00010000);$(norma_st 0 3 0 0 2491 31 00040000);$(norma_st 0 3 0 0 2491 31 00000100);$(norma_st 2 3 0 0 2491 41 00000100);$(norma_st 0 3 0 0 2491 31 00000080);$(norma_st 0 3 0 0 2491 31 80400040)"
    emu_case "flags" -- -d $K --samples 12 -O analog
    # Continuity is only its flag, as C's mask reads it: never set.
    emu_options "status=$(norma_st 0 0 0 0 0001 51);$(norma_st 0 0 0 0 0001 51 01000000)"
    emu_case "continuity" -- -d $K --samples 2 -O analog
    # Digit 4: invalid, overload, both, with a sign; a duplicate is no sample.
    emu_options "status=$(norma_st 0 3 4 0 2491);$(norma_st 0 3 1 0 2491);$(norma_st 0 3 5 0 2491);$(norma_st 0 3 1 4 2491);$(norma_st 0 3 2 0 2491);$(norma_st 0 3 6 0 2491);$(norma_st 0 3 3 0 2491);$(norma_st 0 3 0 0 2491)"
    emu_case "invalid overload duplicate" -- -d $K --samples 5 -O analog
    # A device error is no sample, and an unknown status still is one.
    emu_options "status=$(norma_st 0 3 0 0 2491 34);$(norma_st 0 3 0 0 2491 30);$(norma_st 0 3 0 0 2491 35);$(norma_st 0 3 0 0 2491 39)"
    emu_case "device status" --stderr --filter "$L" -- -d $K --samples 3
    emu_case "device status samples" -- -d $K --samples 3 -O analog
    emu_options "status=$(norma_st 0 3 0 0 2491 32);$(norma_st 0 3 0 0 2491 33)"
    emu_case "input loop and TRANS" -- -d $K --samples 2
    # Hex letters: C's digit conversion gives a to f as 0 to 5.
    emu_options "status=$(norma_st a 0 0 0 0001);$(norma_st f 0 0 0 0001);$S"
    emu_case "value type letters" --oracle-differs "$HEX" --stderr --filter "$L" -- -d $K --samples 2 -O analog
    emu_options "status=$(norma_st 0 a 0 0 0001);$(norma_st 0 f 0 0 0001)"
    emu_case "range letters" --oracle-differs "$HEX" -- -d $K --samples 2 -O analog
    emu_options "status=$(norma_st 0 3 0 0 2491 a1);$(norma_st 0 3 0 0 2491 b1);$(norma_st 0 3 0 0 2491 c1);$(norma_st 0 3 0 0 2491 f1);$S"
    emu_case "mode letters" --oracle-differs "$HEX" --stderr --filter "$L" -- -d $K --samples 4 -O analog
    emu_options "status=$(norma_st 0 3 0 0 2491 3e);$S"
    emu_case "device status letter" --oracle-differs "$HEX" --stderr --filter "$L" -- -d $K --samples 1
    # The first digit of a pair counts for nothing in C.
    emu_options "status=$(norma_st 0 0 0 0 0001 51 20000000)"
    emu_case "continuity beep" --oracle-differs "$PAIR" -- -d $K --samples 1 -O analog
    emu_options "status=$(norma_st 0 3 0 0 2491 31 00800000)"
    emu_case "manual range" --oracle-differs "$PAIR" -- -d $K --samples 1 -O analog
    emu_options "status=$(norma_st 0 3 0 0 2491 31 00001000)"
    emu_case "low battery" -- -d $K --samples 1
    emu_case "low battery warning" --oracle-differs "$PAIR" --stderr --filter "$L" -- -d $K --samples 1

    # Lines gone wrong: each is an error, and the next request goes.
    emu_options "status=abc;$S"
    emu_case "a short line" --stderr --filter "$L" -- -d $K --samples 1
    emu_case "a short line, its sample" -- -d $K --samples 1
    emu_options "status=0303002491310040004x;$S"
    emu_case "not hex" --stderr --filter "$L" -- -d $K --samples 1
    emu_options "status=;$S"
    emu_case "an empty line" --stderr --filter "$L" -- -d $K --samples 1
    # A line longer than 20 digits is cut to them.
    emu_options "status=${S}99;$S"
    emu_options "status=${S}_xy"
    emu_case "digits with a tail that is no digit" -- -d $K --samples 1 -O analog
    emu_options "status=${S}99;$S"
    emu_case "a line too long" --oracle-differs "$CUT" -- -d $K --samples 1 -O analog

    # No answer: a request expires at 2 s, and the next goes.
    emu_options "status=~;$S"
    emu_case "a reply lost" -- -d $K --samples 1
    # Part of a line, and what comes next carries on from it, and C cuts that at 20
    # digits: 24.91 V DC reads as 24.03 with no unit.
    emu_options "status=03030024~;$S"
    emu_case "part of a line" --oracle-differs "$KEEP" -- -d $K --samples 1 -O analog
    # A line the buffer cannot hold never ends for C.
    L100=0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
    emu_options "status=$L100$L100$L100;$S"
    emu_case "a line of 300" --oracle-bug "$FULL_BUF" --oracle-hangs --timeout 4 -- -d $K --samples 1
    # A reply late by about 3 s, with the request after it unanswered. Whichever
    # comes first, the samples are the same. The .tx folds the requests that may or
    # may not run together: when the next goes is where oxrok differs from C, which
    # asks at the late line and is pinned by the driver's tests.
    emu_options late=1:150 "status=$S;~;$(norma_st 0 3 0 0 0001);$S"
    emu_case "a reply late" --tx-filter "s/(5354415455533f0d0a)+/\\1/
$Q1" -- -d $K --samples 2 -O analog
    # Replies with no line feed: the capture never has a line.
    emu_options eol=cr
    emu_case "replies end in CR alone" --no-output --tx-filter "$Q1" -- -d $K --time 1s
    # And oxrok says so at the first expiry.
    emu_case "replies end in CR alone, told" --oracle-differs "$CR" --stderr --filter "$L" --tx-filter "$Q1" -- -d $K --time 2500ms
    # Silent from the start: a time limit ends it.
    emu_options answers=1
    emu_case "silent from the capture on" --no-output --tx-filter "$Q1" -- -d $K --time 300ms

    # A quiet point: the request goes unanswered, the signal lands in its wait.
    emu_options answers=1
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --no-output --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $K --continuous
    emu_options answers=1
    emu_case "unplug in a capture" --oxrok-fails "the port gone" --stderr --filter "$L" --unplug model --timeout 8 --tx-filter '/^unplug$/q' -- -d $K --samples 1
    emu_options
}

# A status from its parts: value type, range, digit 4, sign and first digit, four
# digits, mode and device status (31), the flag pairs (00000000).
norma_st() {
    echo "00$1$2$3$4$5${6:-31}${7:-00000000}"
}

# Statuses for each value type and range, in a reading of 12345.
norma_list() {
    norma_sep=""
    for norma_tr in "$@"; do
        printf '%s%s' "$norma_sep" "$(norma_st "${norma_tr%?}" "${norma_tr#?}" 0 1 2345)"
        norma_sep=";"
    done
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=norma-dmm:conn=$EMU_TTY
    Z=siemens-b102x:conn=$EMU_TTY
}
