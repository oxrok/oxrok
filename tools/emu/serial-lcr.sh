# LCR meters on a serial cable (emu/src/models/serial-lcr.rs), both of sigrok's
# packet formats, on the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a serial model".
EMU_MODEL=serial-lcr
EMU_CDC=1
EMU_DRIVER="deree-de5000 mastech-ms5308 peaktech-2170 uni-t-ut612 peaktech-2165 voltcraft-4080"
EMU_FULL=$EMU_DRIVER
EMU_GCOV=serial-lcr
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/lcr/es51919.c src/lcr/vc4080.c"

emu_conns() {
    D=deree-de5000:conn=$EMU_TTY
    V=voltcraft-4080:conn=$EMU_TTY
}

# An ES51919 packet: flags, config, tolerance, then quantity, value (two bytes),
# info and status of each display.
es_packet() {
    printf '000d%s%s%s%s%s%s%s%s%s%s%s0d0a' "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}"
}

# The default packet with another primary display: quantity, value, info, status.
es_main() {
    es_packet 20 50 00 "$1" "$2" "$3" "$4" 01 007b 03 00
}

# The default packet with other flags and config.
es_flags() {
    es_packet "$1" "$2" 00 02 3039 53 00 01 007b 03 00
}

# A blank primary display and this secondary: quantity, value, info, status.
es_second() {
    es_packet 20 50 00 02 3039 53 01 "$1" "$2" "$3" "$4"
}

# es_sweep NAME FRAMES PACKET...: a capture of these packets, in turn, behind
# the scan's two; FRAMES is how many of them make a frame, and the last must.
# The burst holds what follows, so no count here depends on how long a tool
# listens. ES_FLAGS=stderr compares what is logged instead of the output.
es_sweep() {
    _name=$1
    _frames=$2
    shift 2
    _list=$ES_DEFAULT,$ES_DEFAULT
    for _p in "$@"; do
        _list=$_list,$_p
    done
    emu_options period=5 scan-burst=2 burst=$# packet=$_list
    if [ "$ES_FLAGS" = stderr ]; then
        emu_case "$_name" --stderr --filter "$F" -- -d $D --frames $_frames
    else
        emu_case "$_name" --keep-header -- -d $D --frames $_frames
    fi
}

# A vc4080 packet as text: the 27 characters from the function to the Q range,
# then the 10 flags.
vc_packet() {
    emu_hex "$1$2\r\n"
}

# vc_sweep NAME FRAMES PACKET...: as es_sweep. A packet is asked for every
# 500 ms, so these are short.
vc_sweep() {
    _name=$1
    _frames=$2
    shift 2
    _list=$VC_DEFAULT,$VC_DEFAULT
    for _p in "$@"; do
        _list=$_list,$_p
    done
    emu_options meter=vc4080 packet=$_list
    if [ "$VC_FLAGS" = stderr ]; then
        emu_case "$_name" --driver voltcraft-4080 --stderr --filter "$F" -- -d $V --frames $_frames
    else
        emu_case "$_name" --driver voltcraft-4080 --keep-header -- -d $V --frames $_frames
    fi
}

# The seven main display ranges of a function and frequency.
vc_ranges() {
    for _r in 0 1 2 3 4 5 6; do
        vc_packet "$1Q$2SA12345${_r}0134210711414062" "__________"
        printf ' '
    done
}

# The default packet with other flags.
vc_flagged() {
    vc_packet "LQAPA1234530134210711414062" "$1"
}

# The default packet with another main value and secondary value.
vc_value() {
    vc_packet "LQAPA${1}3${2}210711414062" "__________"
}

# With a secondary of this function letter, main range and secondary range.
vc_second() {
    vc_packet "$1ASA12345${2}0134${3}10711414062" "__________"
}

emu_cases() {
    emu_conns
    ES_DEFAULT=$(es_flags 20 50)
    VC_DEFAULT=$(vc_flagged __________)
    ES_FLAGS=
    VC_FLAGS=
    # oxrok's log prefix, on its side only.
    F='s/^oxrok: [a-z]+: oxrok::lcr::es51919: /sr: es51919: /
s/^oxrok: [a-z]+: oxrok::lcr::vc4080: /sr: vc4080: /
s/^oxrok: [a-z]+: oxrok::drivers::serial_lcr: /sr: serial-lcr: /'
    HUP_BUG="lcr_receive_data takes a hung up port for a timeout (protocol.c:193-196: revents is not exactly G_IO_IN), and neither a meter that is not asked nor vc4080_packet_request, which ignores its write, can fail there, so it never ends at a frame limit; oxrok ends the capture"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # What is on the line, for each id of the table.
    emu_case "es show" --keep-header -- -d $D --show
    for _id in deree-de5000 mastech-ms5308 peaktech-2170 uni-t-ut612; do
        emu_case "scan $_id" --driver $_id --keep-header -- -d $_id:conn=$EMU_TTY --scan
    done
    emu_case "es no conn" --keep-header --no-output -- -d deree-de5000 --scan
    emu_case "es no such node" --keep-header --no-output -- -d deree-de5000:conn=/dev/ttyACM97 --scan
    emu_case "es wrong baud" --keep-header --no-output -- -d $D:serialcomm=1200/8n1 --scan
    emu_case "es wrong frame" --keep-header --no-output -- -d $D:serialcomm=9600/7e1 --scan
    emu_case "es no such serialcomm" --keep-header --no-output --tx-filter '/^ctl_out/d' -- -d $D:serialcomm=fast --scan
    emu_case "es serialcomm lines down" --keep-header -- -d $D:serialcomm=9600/8n1/rts=0/dtr=0 --scan

    # The keys.
    emu_case "es get freq" -- -d $D --get output_frequency
    emu_case "es get model" -- -d $D --get equiv_circuit_model
    emu_case "es get limit_frames" -- -d $D --get limit_frames
    emu_case "es get limit_time" --expect-fail --stderr -- -d $D --get limit_time
    emu_case "es get conn" --expect-fail --stderr -- -d $D --get conn
    emu_case "es get samplerate" --expect-fail --stderr -- -d $D --get samplerate
    emu_case "es set freq" --expect-fail --stderr -- -d $D --set -c output_frequency=100
    emu_case "es samples" --expect-fail --stderr -- -d $D --samples 1
    emu_options period=5 scan-burst=2 burst=1
    emu_case "es limit_frames" --keep-header -- -d $D -c limit_frames=1 --time 5s
    emu_case "es limit_frames 0" --keep-header -- -d $D --frames 0 --time 800ms
    emu_case "es time" --keep-header -- -d $D --time 800ms
    emu_options

    # A scan's second listen hears nothing: it found the meter, knows no values.
    emu_options scan-burst=1 burst=0
    emu_case "es no values show" --keep-header -- -d $D --show
    emu_case "es no values get freq" -- -d $D --get output_frequency
    emu_case "es no values get model" --expect-fail --stderr -- -d $D --get equiv_circuit_model
    emu_options burst=0
    emu_case "es never sends" --keep-header --no-output -- -d $D --scan
    emu_options

    # A capture, plain: what the stream does with no silence to help.
    emu_case "es frames 1" --keep-header -- -d $D --frames 1
    emu_case "es frames 2" --keep-header -- -d $D --frames 2
    emu_options scan-burst=2 burst=2
    emu_case "es frames 2 csv" -- -d $D --frames 2 -O csv:header=false
    emu_options scan-burst=2 burst=1
    emu_case "es P1 alone" --keep-header -- -d $D -C P1 --frames 1
    emu_case "es P2 alone" --keep-header -- -d $D -C P2 --frames 1
    emu_case "es renamed" --keep-header -- -d $D -C P1=main,P2 --frames 1
    # The limit is checked once all of a read is out: a read holding two
    # packets sends both.
    emu_options scan-burst=2 burst=1 noise=$ES_DEFAULT
    emu_case "es a read of two packets" --keep-header -- -d $D --frames 1
    emu_options

    # Each side of the packet's fields.
    es_sweep "es units" 15 $(for _u in 02 0a 12 1a 22 2a 32 3a 42 4a 52 5a 62 6a 72; do es_main 02 04d2 $_u 00; printf ' '; done)
    ES_FLAGS=stderr
    es_sweep "es unknown units" 2 $(es_main 02 04d2 7a 00) $(es_main 02 04d2 fa 00)
    ES_FLAGS=
    es_sweep "es decimals" 8 $(for _d in 50 51 52 53 54 55 56 57; do es_main 02 04d2 $_d 00; printf ' '; done)
    # The unit no line of the table names, no prefix unlike "no unit".
    es_sweep "es large with unit 4" 2 $(es_main 02 7fff 20 00) $(es_main 02 7fff 00 00)
    es_sweep "es values" 7 $(for _v in 0000 0001 7fff 8000 ffff ff85 4e20; do es_main 02 $_v 53 00; printf ' '; done)
    es_sweep "es primary quantity" 4 $(for _q in 01 02 03 04; do es_main $_q 04d2 53 00; printf ' '; done)
    es_sweep "es primary parallel" 4 $(for _q in 01 02 03 04; do es_packet 80 50 00 $_q 04d2 53 00 03 007b 0b 00; printf ' '; done)
    ES_FLAGS=stderr
    es_sweep "es primary unknown quantity" 3 $(es_main 00 04d2 53 00) $(es_main 05 04d2 53 00) $(es_main ff 04d2 53 00)
    ES_FLAGS=
    es_sweep "es secondary quantity" 5 $(for _q in 00 01 02 03 04; do es_second $_q 007b 03 00; printf ' '; done)
    ES_FLAGS=stderr
    es_sweep "es secondary unknown quantity" 2 $(es_packet 20 50 00 02 3039 53 00 05 007b 03 00) $(es_packet 20 50 00 02 3039 53 00 ff 007b 03 00)
    ES_FLAGS=
    es_sweep "es status" 12 $(for _s in 00 01 02 03 04 07 08 09 0a 0f 10 13; do es_main 02 04d2 53 $_s; printf ' '; done)
    es_sweep "es secondary status" 3 $(es_second 01 007b 03 00) $(es_second 01 007b 03 01) $(es_second 01 007b 03 03) $(es_second 01 007b 03 83)
    es_sweep "es outside limits" 2 $(es_main 02 4e20 53 03) $(es_main 02 ffff 53 03)
    es_sweep "es flags" 6 $(for _f in 00 01 02 03 04 20; do es_flags $_f 50; printf ' '; done)
    # Neither display in these modes: no frame, no count, no META.
    es_sweep "es calibration and sorting" 1 $(es_flags 08 50) $(es_flags 10 50) $(es_flags 18 10) $(es_flags 20 50)
    es_sweep "es frequencies" 6 $(for _c in 10 30 50 70 90 b0; do es_flags 20 $_c; printf ' '; done)
    ES_FLAGS=stderr
    es_sweep "es unknown frequencies" 2 $(es_flags 20 d0) $(es_flags 20 f0)
    ES_FLAGS=
    es_sweep "es models" 5 $(es_flags 00 50) $(es_flags 80 50) $(es_flags 40 50) $(es_flags c0 50) $(es_main 04 04d2 0b 00)

    # The scan's packets are parsed too: what is unknown is logged there.
    emu_options period=5 scan-burst=2 burst=1 packet=$(es_main 05 04d2 53 00)
    emu_case "es unknown quantity in the scan" --stderr --filter "$F" -- -d $D --scan
    emu_options period=5 scan-burst=2 burst=1 packet=$(es_flags 20 f0)
    emu_case "es unknown frequency in the scan" --stderr --filter "$F" -- -d $D --scan
    emu_options period=5 scan-burst=2 burst=1 packet=$(es_flags 40 50)
    emu_case "es scan finds auto" -- -d $D --get equiv_circuit_model

    # Noise on the line. A packet search gives up after 128 bytes, and warns
    # over two packets dropped.
    emu_options noise=3f3f3f scan-burst=2 burst=2
    emu_case "es noise before each packet" --keep-header -- -d $D --frames 2
    emu_options noise=$(printf '3f%.0s' $(seq 1 35)) scan-burst=2 burst=1
    emu_case "es noise of two packets" --stderr --filter "$F" -- -d $D --scan
    emu_options noise=$(printf '3f%.0s' $(seq 1 34)) scan-burst=2 burst=1
    emu_case "es noise just under two packets" --keep-header -- -d $D --scan
    emu_options noise=$(printf '3f%.0s' $(seq 1 111)) scan-burst=2 burst=1
    emu_case "es noise up to the buffer" --keep-header -- -d $D --scan
    emu_options noise=$(printf '3f%.0s' $(seq 1 112)) scan-burst=1 burst=0
    emu_case "es noise past the buffer" --keep-header --no-output -- -d $D --scan
    emu_options packet=$(printf '00%.0s' $(seq 1 17)) burst=1
    emu_case "es no packet" --keep-header --no-output -- -d $D --scan
    emu_options noise=000d$(printf '00%.0s' $(seq 1 15)) scan-burst=2 burst=1
    emu_case "es a header with no footer" --keep-header -- -d $D --frames 1

    # The USB side.
    emu_options chunk=1 scan-burst=2 burst=1
    emu_case "es one byte per transfer" --keep-header -- -d $D --frames 1
    emu_options stall=line
    emu_case "es line coding stalled" --keep-header -- -d $D --scan
    emu_options stall=lines
    emu_case "es control lines stalled" --keep-header --no-output --tx-filter '/^ctl_out 21 20 /d' -- -d $D --scan
    emu_options in-stall=1
    emu_case "es a stalled read" --keep-header -- -d $D --scan
    emu_options

    # An unplug with the tty open. The mark is a period after the packet.
    emu_options scan-burst=2 burst=1
    emu_case "es unplug" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 8 -- -d $D --frames 5
    # Silence at a frame limit: no bound in either tool, as the meter may come
    # back. A signal is what ends it.
    emu_case "es interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $D --frames 5
    emu_case "es lib silent at a frame limit" --libsigrok --hangs --timeout 3 -- -d $D --frames 5
    emu_options

    # The same for the meter that is asked.
    emu_options meter=vc4080
    emu_case "vc show" --driver voltcraft-4080 --keep-header -- -d $V --show
    for _id in peaktech-2165 voltcraft-4080; do
        emu_case "scan $_id" --driver $_id --keep-header -- -d $_id:conn=$EMU_TTY --scan
    done
    emu_case "vc no conn" --driver voltcraft-4080 --keep-header --no-output -- -d voltcraft-4080 --scan
    emu_case "vc wrong baud" --driver voltcraft-4080 --keep-header --no-output -- -d $V:serialcomm=9600/8n1 --scan
    emu_case "vc the 7e1 its documents say" --driver voltcraft-4080 --keep-header --no-output -- -d $V:serialcomm=1200/7e1 --scan
    emu_case "vc get freq" --driver voltcraft-4080 -- -d $V --get output_frequency
    emu_case "vc get model" --driver voltcraft-4080 -- -d $V --get equiv_circuit_model
    emu_case "vc frames 1" --driver voltcraft-4080 --keep-header -- -d $V --frames 1
    emu_case "vc frames 2" --driver voltcraft-4080 --keep-header -- -d $V --frames 2
    emu_case "vc frames 2 csv" --driver voltcraft-4080 -- -d $V --frames 2 -O csv:header=false
    emu_case "vc P1 alone" --driver voltcraft-4080 --keep-header -- -d $V -C P1 --frames 1
    emu_case "vc P2 alone" --driver voltcraft-4080 --keep-header -- -d $V -C P2 --frames 1
    emu_case "vc time" --driver voltcraft-4080 --keep-header -- -d $V --time 300ms
    emu_options meter=vc4080 parity=0
    emu_case "vc no parity bits" --driver voltcraft-4080 --keep-header -- -d $V --frames 1
    emu_options meter=vc4080 uart=1200/7e1
    emu_case "vc on a 7e1 line" --driver voltcraft-4080 --keep-header -- -d $V:serialcomm=1200/7e1 --frames 1

    # Asked and silent.
    emu_options meter=vc4080 answers=0
    emu_case "vc never answers" --driver voltcraft-4080 --keep-header --no-output -- -d $V --scan
    emu_options meter=vc4080 answers=1
    emu_case "vc no values show" --driver voltcraft-4080 --keep-header -- -d $V --show
    emu_case "vc no values get model" --driver voltcraft-4080 --expect-fail --stderr -- -d $V --get equiv_circuit_model
    # Asks again every 500 ms: how many goes by the clock.
    emu_options meter=vc4080 answers=2
    emu_case "vc stops answering" --driver voltcraft-4080 --keep-header --no-output \
        --tx-filter 's/^bulk_out 02 (4e)+ -> [0-9]+$/bulk_out 02 4e... -> N/' -- -d $V --time 1200ms

    emu_options meter=vc4080 answers=3
    emu_case "vc unplug" --driver voltcraft-4080 --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 8 -- -d $V --frames 5
    emu_case "vc interrupt" --driver voltcraft-4080 --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $V --frames 5

    # Each side of the packet's fields.
    for _f in "L A" "L B" "C A" "C B" "R A"; do
        vc_sweep "vc ranges $_f" 7 $(vc_ranges ${_f% *} ${_f#* })
    done
    # No such range, or a function or frequency with no table.
    vc_sweep "vc no such main range" 1 $(vc_packet "LQAPA1234570134210711414062" "__________") $VC_DEFAULT
    vc_sweep "vc unknown frequency" 3 $(vc_packet "LQZPA1234530134210711414062" "__________") $(vc_packet "RRZPA1234530134210711414062" "__________") $VC_DEFAULT
    vc_sweep "vc unknown function" 1 $(vc_packet "XQAPA1234530134210711414062" "__________") $(vc_packet "LXAPA1234530134210711414062" "__________") $VC_DEFAULT
    # D, Q, none, and the resistance of each Rs: the main range picks it.
    vc_sweep "vc secondary D Q" 8 $(for _r in 1 2 3 4; do vc_second LD 3 $_r; printf ' '; vc_second CQ 3 $_r; printf ' '; done)
    vc_sweep "vc secondary out of range" 4 $(for _r in 0 5 6 9; do vc_second LQ 3 $_r; printf ' '; done)
    vc_sweep "vc secondary R Rs 100" 5 $(for _r in 1 2 3 4 5; do vc_second LR 0 $_r; printf ' '; done)
    vc_sweep "vc secondary R Rs 1k" 5 $(for _r in 1 2 3 4 5; do vc_second LR 3 $_r; printf ' '; done)
    vc_sweep "vc secondary R Rs 10k" 5 $(for _r in 1 2 3 4 5; do vc_second CR 2 $_r; printf ' '; done)
    vc_sweep "vc secondary R Rs 100k" 5 $(for _r in 1 2 3 4 5; do vc_second LR 5 $_r; printf ' '; done)
    # D and Q have their own ranges, which no channel uses.
    vc_sweep "vc D and Q ranges no such" 1 $(vc_packet "LQAPA1234530134210711!1406!" "__________")
    vc_sweep "vc no secondary" 1 $(vc_packet "R_A_A1234530134210711414062" "__________")
    vc_sweep "vc models" 5 $(vc_packet "LQAPA1234530134210711414062" "__________") $(vc_packet "LQASA1234530134210711414062" "__________") $(vc_packet "LQAXA1234530134210711414062" "__________") $(vc_packet "RRAPA1234530134210711414062" "__________") $(vc_packet "CQBSM1234530134210711414062" "__________")
    # Values: switching ranges, over the range, and C's text to number.
    vc_sweep "vc switching and over" 3 $(vc_value 80000 0134) $(vc_value 90000 0134) $(vc_value 12345 9134) $(vc_value 12345 0134)
    vc_sweep "vc numbers" 8 $(vc_value 12a34 12a4) $(vc_value '\0401.50' '\0401.5') $(vc_value 1.5e1 1e+1) $(vc_value +1234 -123) $(vc_value 0x1f0 0x1f) $(vc_value '12\00045' '1\00034') $(vc_value 1e999 9e99) $(vc_value 12345 '\000\000\000\000')
    vc_sweep "vc numbers spelled" 3 $(vc_value 'inf\040\040' 'inf\040') $(vc_value infin -inf) $(vc_value 12345 'inf\000')
    # Flags, one per packet.
    vc_sweep "vc setup and unknown" 1 $(vc_flagged S_________) $(vc_flagged X_________) $(vc_flagged __________)
    vc_sweep "vc hold and fuse" 3 $(vc_flagged _F________) $(vc_flagged __H_______) $(vc_flagged _FH_______)
    vc_sweep "vc min max" 5 $(for _m in R M I A X _; do vc_flagged ___${_m}______; printf ' '; done)
    vc_sweep "vc min max unknown" 1 $(vc_flagged ___Q______) $(vc_flagged __________)
    vc_sweep "vc relative" 2 $(vc_flagged ____R_____) $(vc_flagged ____S_____) $(vc_flagged ____X_____) $(vc_flagged __________)
    vc_sweep "vc limits and tolerance" 1 $(vc_flagged _____L____) $(vc_flagged ______T___) $(vc_flagged ______S___) $(vc_flagged __________)
    vc_sweep "vc backlight and adapter" 3 $(vc_flagged _______B__) $(vc_flagged ________A_) $(vc_flagged _______BA_)
    VC_FLAGS=stderr
    vc_sweep "vc low battery" 1 $(vc_flagged _________B)
    VC_FLAGS=
    vc_sweep "vc low battery output" 1 $(vc_flagged _________B)

    # A packet's terminator is all the check there is.
    vc_sweep "vc anything that ends well" 1 $(vc_packet '?????????????????????????????????????' '') $VC_DEFAULT
    emu_options meter=vc4080 noise=3f3f3f
    emu_case "vc noise before each packet" --driver voltcraft-4080 --keep-header -- -d $V --frames 2
    emu_options meter=vc4080 noise=$(printf '3f%.0s' $(seq 1 79))
    emu_case "vc noise of two packets" --driver voltcraft-4080 --stderr --filter "$F" -- -d $V --scan
    emu_options meter=vc4080 noise=$(printf '3f%.0s' $(seq 1 90))
    emu_case "vc noise past the buffer" --driver voltcraft-4080 --keep-header --no-output -- -d $V --scan
    emu_options meter=vc4080 chunk=1
    emu_case "vc one byte per transfer" --driver voltcraft-4080 --keep-header -- -d $V --frames 1
    emu_options
}
