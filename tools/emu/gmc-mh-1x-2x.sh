# A Gossen Metrawatt METRAHit (emu/src/models/gmc-mh-1x-2x.rs) on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial
# model". C registers two drivers: gmc-mh-1x-2x-rs232 for a meter in send mode,
# which only listens, and gmc-mh-2x-bd232 for one that answers requests. A scan
# of the second takes 5 s whatever the device does (three writes a second apart
# to wake a meter that is off, then 2 s for meters to answer), so each of its
# cases costs 10 s or more, and one case holds as many answers as it can.
EMU_MODEL=gmc-mh-1x-2x
EMU_CDC=1
EMU_DRIVER="gmc-mh-1x-2x-rs232 gmc-mh-2x-bd232"
EMU_FULL=$EMU_DRIVER
EMU_GCOV=gmc-mh-1x-2x
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# Both scans return with the port open (api.c:209, :304, no serial_close), and
# dev_open opens it again, which libserialport's exclusive open refuses from
# 0.1.2 (2024-09-10), and this oracle builds later than that. So stock sigrok-cli
# runs only --scan here. On 0.1.1 the driver works and users there reach the rest,
# so the cases run sigrok-cli with locking disabled, the C driver unmodified, as
# tools/emu/gwinstek-gpd.sh does; the "stock" cases run without it.
gm_openable() {
    GM_STOCK_CLI=$SIGROK_CLI
    [ "$no_oracle" = 1 ] && return
    printf '%s\n' '#define _GNU_SOURCE' '#include <dlfcn.h>' '#include <stdarg.h>' \
        '#include <sys/ioctl.h>' \
        'int flock(int fd, int op) { (void)fd; (void)op; return 0; }' \
        'int ioctl(int fd, unsigned long req, ...) {' \
        '    static int (*real)(int, unsigned long, void *);' \
        '    va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);' \
        '    if (req == TIOCEXCL) return 0;' \
        '    if (!real) real = (int (*)(int, unsigned long, void *))dlsym(RTLD_NEXT, "ioctl");' \
        '    return real(fd, req, arg);' \
        '}' >"$TMP/gm-openable.c"
    cc -shared -fPIC -o "$TMP/gm-openable.so" "$TMP/gm-openable.c" -ldl 2>"$TMP/gm-openable.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    printf '%s\n' '#!/bin/sh' \
        "LD_PRELOAD=$TMP/gm-openable.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$SIGROK_CLI") \"\$@\"" \
        >"$TMP/gm-sigrok-cli"
    chmod +x "$TMP/gm-sigrok-cli"
    SIGROK_CLI=$TMP/gm-sigrok-cli
}

# Digits as nibbles, lowest first: 654321 is 123456.
gm_digits() { printf %s "$1" | sed 's/./3&/g'; }
# Send mode's 13 bytes: CODE FUNCTION RANGE SPECIAL DIGITS [INTERVAL].
gm13() {
    printf '%02x%02x%02x%02x%02x' "$1" $((0x30 | ($2 & 15))) $((0x30 | ($4 & 15))) \
        $((0x30 | (($4 >> 4) & 15))) $((0x30 | $3))
    gm_digits "$5"
    printf '%02x%02x' $((0x30 | (($2 >> 4) & 15))) $((0x30 | ${6:-3}))
}
# Send mode's 5 bytes: CODE FUNCTION RANGE SPECIAL.
gm5() {
    printf '%02x%02x%02x%02x%02x' "$1" $((0x30 | $2)) $((0x30 | ($4 & 15))) \
        $((0x30 | (($4 >> 4) & 15))) $((0x30 | $3))
}
# Send mode's 6 bytes: ID RANGE DIGITS (five), ID being 1 or 2 in bits 5 and 4.
gm6() { printf '%02x' $((($1 << 4) | $2)); gm_digits "$3"; }
# The 10 byte block: CODE FUNCTION RANGE SPECIAL DIGITS (five).
gm10() { gm5 "$1" "$2" "$3" "$4"; gm_digits "$5"; }
# A measurement's nine bytes: FUNCTION RANGE DIGITS (RANGE with 0x10 for valid).
gm_meas() { printf '00%02x%02x' "$1" "$2"; printf %s "$3" | sed 's/./0&/g'; }
# A status's nine bytes: FUNCTION RANGE TYPE.
gm_status() { printf '0201%02x%02x%02x00001e%02x' 3 "$1" "$2" "$3"; }
# A whole answer: its 14 bytes from the 13 given in hex, with the checksum.
gm_frame() {
    gm_sum=0
    gm_rest=$1
    while [ -n "$gm_rest" ]; do
        gm_sum=$((gm_sum + 0x$(printf %s "$gm_rest" | cut -c1-2)))
        gm_rest=$(printf %s "$gm_rest" | cut -c3-)
    done
    printf '%s%02x' "$1" $(((64 - gm_sum) & 63))
}

emu_cases() {
    gm_openable
    emu_conns
    S=$(gm13 1 1 1 0 654321)
    # oxrok's log prefix, on its side only.
    L='s/^oxrok: (err|warn|info): oxrok::drivers::gmc_mh_1x_2x(::protocol)?: /sr: gmc-mh-1x-2x: /'
    LOCK_BUG="api.c:209 and :304 return with the scan's port open, so dev_open's second open is refused by the exclusive open of libserialport 0.1.2 and later (serialport.c:589, :599): every path but --scan fails on stock sigrok-cli there, and works on 0.1.1; oxrok closes the port after the scan and opens it again"
    REGRESSION="f4bd8a17 (2016, properly set encoding digits) made scale an exponent from 0 and scale *= -1.0 became scale = -scale; until then it negated the value"
    SIGN_BUG="$REGRESSION. protocol.c:255-257 and :506-508 now do nothing, as scale is 0 there: a negative reading is positive. D3 of the range byte is the value's sign in the manufacturer's table TR, in command 8 and in C before that commit; oxrok negates the value"
    SIGN16_BUG="$REGRESSION. protocol.c:112-113 now does nothing, as scale is 0 there; 1d4a2839 (2014) had moved a 12S to 16S's sign to this bit: a negative reading is positive; oxrok negates the value, as C did"
    EXP_BUG="$REGRESSION. protocol.c:506-508 now makes scale positive for a sign bit of 0 wherever decode_ctmv_2x has put something there (:381, :399, :402, :416), so a capacitance reads 100 times high, a continuity voltage 10^10 and a power 10^2 to 10^4; oxrok leaves the exponent"
    CURRENT_BUG="$REGRESSION. protocol.c:593-595 does the same after adding the range, where scale is -3 to -6, so any positive current read by command 8 is 10^6 to 10^12 high (300 uA reads 300 MA), and :506-508 does it to a capacitance; a negative one has no sign. oxrok leaves the exponent and negates the value"
    FUSE_BUG="protocol.c:1056-1057 logs a broken fuse and :1064 adds the digit in as 13, a reading that is no measurement; oxrok sends no number, as for OL"
    OL18_BUG="protocol.c:756-758 adds 1010 in as a digit in a 10 byte block; the manufacturer's digit table has it OL, and a 2x sends this block under the 18S's code in one mode; oxrok sends no number for that code"
    SECOND_BUG="api.c:294-296 gives the next meter a new sdi with no priv, and chk_msg14 returns at protocol.c:915 with no message, so only the first meter that answers is found though the comment at api.c:265 waits for device(s); oxrok lists each"
    SHIFT_BUG="protocol.c:1196-1201 frames answers by counting 14 and buflen survives the expiry at :1212-1213, so after a lost byte every block straddles two answers and fails its checksum, and no sample follows; oxrok finds the next header after a failed check"
    LATE_BUG="protocol.c:1212-1221 asks again at each second and at each answer, and takes every answer as the reading, so at a latency over 1 s what is owed grows and samples get old; oxrok counts what is owed, lets one go unused that has another owed behind it, and waits longer. So a lost answer costs it 3 s where it costs C 1 s"
    SILENT_BUG="protocol.c:1212-1221 asks again every second for ever, so a --samples capture never ends on a meter that stopped answering; oxrok fails it after 10 s with no usable answer"
    HUP_BUG="gmc_mh_1x_2x_receive_data reads only when revents == G_IO_IN (protocol.c:1102), which a hung up port is not, and in send mode it writes nothing that could fail: a --samples capture never ends; oxrok's failed read ends it"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    # What varies with time in a wait: requests going out again.
    F_REQUESTS='s/^bulk_out 02 (ff0f00[0-9a-f]{78})+ -> [0-9]+$/bulk_out 02 requests/'

    # Send mode. The scan takes a block's first five bytes and goes on to the
    # capture with the next.
    emu_options mode=send
    emu_case "scan" --keep-header -- -d $K --scan
    emu_case "show" --keep-header -- -d $K --show
    emu_case "get limit_samples" -- -d $K --get limit_samples
    emu_case "get limit_time" -- -d $K --get limit_time
    emu_case "set then get limit_samples" -- -d $K -c limit_samples=7 --get limit_samples
    emu_case "get conn" --expect-fail --stderr -- -d $K --get conn
    # Not listed in send mode: check_key refuses it ahead of the driver.
    emu_case "set power_off in send mode" --expect-fail --stderr --filter "$L" -- -d $K -c power_off=1 --set
    emu_case "no such node" --keep-header --no-output -- -d gmc-mh-1x-2x-rs232:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d gmc-mh-1x-2x-rs232 --scan
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $K:serialcomm=fast --scan
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=9600/6n1 --scan
    # Each code a meter's name, the 13 bytes being what the scan reads.
    for gm_code in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
        emu_options mode=send msgs=$(gm5 $gm_code 1 1 0)
        emu_case "scan code $gm_code" --keep-header -- -d $K --scan
    done
    # The older adapter's rate: nothing at the first, found at the second.
    emu_options mode=send uart=8228/6n1 msgs=$(gm5 4 2 1 0)
    emu_case "scan at the second rate" --keep-header -- -d $K --scan
    emu_case "serialcomm given is the only one tried" --keep-header --no-output -- -d $K:serialcomm=9600/6n1 --scan
    emu_case "serialcomm 8228" --keep-header -- -d $K:serialcomm=8228/6n1 --scan
    # What is no meter.
    emu_options mode=send burst=0
    emu_case "scan silent" --keep-header --no-output -- -d $K --scan
    emu_case "scan silent one rate" --keep-header --no-output -- -d $K:serialcomm=9600/6n1 --scan
    emu_options mode=send msgs=00 burst=1
    emu_case "scan unknown code" --keep-header --stderr --filter "$L" -- -d $K:serialcomm=9600/6n1 --scan
    emu_options mode=send msgs=0131 burst=1
    emu_case "scan short of data" --keep-header --no-output -- -d $K:serialcomm=9600/6n1 --scan
    emu_options mode=send msgs=0131303031 burst=1
    emu_case "scan four data bytes" --keep-header -- -d $K:serialcomm=9600/6n1 --scan
    emu_options mode=send msgs=013130301131 burst=1
    emu_case "scan a start in the data" --keep-header --no-output -- -d $K:serialcomm=9600/6n1 --scan
    # 99 bytes ahead of the start, and 100: the scan reads 100.
    gm_many=$(printf '30%.0s' $(seq 1 99))
    emu_options mode=send msgs=${gm_many}0131303031 burst=1
    emu_case "scan start at the last byte" --keep-header -- -d $K:serialcomm=9600/6n1 --scan
    emu_options mode=send msgs=${gm_many}300131303031 burst=1
    emu_case "scan start past the last byte" --keep-header --no-output -- -d $K:serialcomm=9600/6n1 --scan
    emu_options mode=send msgs=$(printf 'c1%.0s' 1)f1f0f0f1 burst=1 uart=9600/8n1
    emu_case "scan high bits" --keep-header -- -d $K:serialcomm=9600/8n1 --scan
    emu_options mode=send stall=line
    emu_case "line coding stalled" --keep-header --no-output -- -d $K --scan
    emu_options mode=send stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # No in-stall= case: a meter that is not asked again loses its block by how the
    # kernel's recovery falls, and a scan then finds nothing, or finds it, at random.

    # A capture. A block sent as one holds as many samples as --samples says, as
    # the limit is looked at when the bytes run out.
    emu_options mode=send
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2 analog" -- -d $K --samples 2 -O analog
    emu_case "samples csv" -- -d $K --samples 1 -O csv:header=false:time=false
    emu_case "channel P1" -- -d $K -C P1 --samples 1
    emu_case "no such channel" --stderr --filter 's/^Failed to set channels: invalid argument: //' --expect-fail -- -d $K -C P2 --samples 1
    # Far past the capture's one block, which comes a period after the open.
    emu_options mode=send blocks=2
    emu_case "time" -- -d $K --time 3s

    # Every function of a 2x, with and without RMS (24S and up have it).
    for gm_code in 1 2; do
        gm_block=
        gm_n=0
        for gm_f in 0 1 2 3 4 5 6 7 8 10 11 12 15 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32; do
            gm_block=$gm_block$(gm13 $gm_code $gm_f 1 0 654321)
            gm_n=$((gm_n + 1))
        done
        emu_options mode=send msgs=$(gm5 $gm_code 1 1 0)30,$gm_block
        emu_case "2x functions code $gm_code" -- -d $K --samples $gm_n -O analog
    done
    # A function past the table's.
    emu_options mode=send msgs=$S,$(gm13 1 33 1 0 654321)$(gm13 1 255 1 0 654321)$(gm13 1 30 1 0 654321)
    emu_case "2x function out of the table" --stderr --filter "$L" -- -d $K --samples 3
    emu_case "2x function out of the table, its output" -- -d $K --samples 3 -O analog
    # Every range of what has them.
    for gm_f in 1 4 6 8 10 11 18 15 17; do
        gm_block=
        for gm_r in 0 1 2 3 4 5 6 7; do
            gm_block=$gm_block$(gm13 1 $gm_f $gm_r 0 654321)
        done
        emu_options mode=send msgs=$S,$gm_block
        emu_case "2x ranges function $gm_f" -- -d $K --samples 8 -O analog
    done
    # The special characters: DATA, MAN, and the rest that say nothing.
    emu_options mode=send msgs=$S,$(gm13 1 1 1 16 654321)$(gm13 1 1 1 128 654321)$(gm13 1 1 1 144 654321)$(gm13 1 1 1 111 654321)
    emu_case "2x special characters" -- -d $K --samples 4 -O analog
    # OL on any digit, and the digits past it, which are reserved.
    emu_options mode=send msgs=$S,$(gm13 1 1 1 0 a54321)$(gm13 1 1 1 0 65432a)$(gm13 1 1 1 0 b54321)$(gm13 1 1 1 0 f54321)$(gm13 1 4 1 0 a54321)
    emu_case "2x overload" -- -d $K --samples 5 -O analog
    # Every send interval, none of which shows.
    gm_block=
    for gm_i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do gm_block=$gm_block$(gm13 1 1 1 0 654321 $gm_i); done
    emu_options mode=send msgs=$S,$gm_block
    emu_case "2x send intervals" -- -d $K --samples 16
    # A 2x's 5 and 6 bytes, for V DC and A DC at its fastest. A block starts the
    # next one's bytes, so the last start has nothing to end it.
    emu_options mode=send msgs=$S,$(gm5 1 1 1 0)$(gm6 1 1 12345)$(gm6 2 1 54321)$(gm5 1 6 1 128)$(gm6 1 1 12345)$(gm6 1 2 a2345)
    emu_case "2x five and six" -- -d $K --samples 4 -O analog
    emu_options mode=send msgs=$S,$(gm5 2 1 1 0)$(gm6 1 1 12345)
    emu_case "another meter's code" --stderr --filter "$L" -- -d $K --samples 1
    emu_options mode=send msgs=$S,$(gm13 13 1 1 0 654321)$(gm13 0 1 1 0 654321)
    emu_case "other codes in a 13 byte block" --stderr --filter "$L" -- -d $K --samples 2

    # Metrahit 12 to 16: 5 and 6 bytes, and 15 up 10 bytes. Scale of the range.
    for gm_code in 4 8 9 10 11 6 7; do
        gm_block=
        gm_n=0
        for gm_f in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
            for gm_r in 0 1 2 3; do
                gm_block=$gm_block$(gm10 $gm_code $gm_f $gm_r 0 12345)
                gm_n=$((gm_n + 1))
            done
        done
        emu_options mode=send msgs=$(gm5 $gm_code 1 1 0)30,$gm_block
        emu_case "1x functions code $gm_code" -- -d $K --samples $gm_n -O analog
    done
    gm_block=
    for gm_f in 1 2 4 5 7 9 10 11 12 14 15; do
        for gm_r in 0 1 8 11; do
            gm_block=$gm_block$(gm5 11 $gm_f $gm_r 0)$(gm6 1 $gm_r 12345)
        done
    done
    emu_options mode=send msgs=$(gm5 11 1 1 0)30,$gm_block$(gm5 11 1 1 0)
    emu_case "1x five and six" -- -d $K --samples 44 -O analog
    # The bits: sign, AC for a current, and the special characters.
    emu_options mode=send msgs=$(gm5 11 1 1 0)30,$(gm10 11 2 1 16 12345)$(gm10 11 2 1 64 12345)$(gm10 11 2 1 32 12345)$(gm10 11 2 1 128 12345)$(gm10 11 2 5 0 12345)$(gm10 11 15 9 0 12345)$(gm10 11 11 0 0 12345)
    emu_case "1x special characters and sign" --oracle-differs "$SIGN16_BUG" -- -d $K --samples 7 -O analog
    # OL on 10 bytes is 12 and up, and 11 an empty digit; on 6 bytes 10 and up.
    emu_options mode=send msgs=$(gm5 11 1 1 0)30,$(gm10 11 2 1 0 c2345)$(gm10 11 2 1 0 b2345)$(gm10 11 2 1 0 a2345)$(gm5 11 2 1 0)$(gm6 1 1 a2345)$(gm6 1 1 b2345)$(gm5 11 2 1 0)
    emu_case "1x overload" -- -d $K --samples 5 -O analog

    # Metrahit 18: its tables.
    gm_block=
    gm_n=0
    for gm_f in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
        for gm_r in 0 3 7; do
            gm_block=$gm_block$(gm10 13 $gm_f $gm_r 0 12345)
            gm_n=$((gm_n + 1))
        done
    done
    emu_options mode=send msgs=$(gm5 13 1 1 0)30,$gm_block
    emu_case "18 functions" -- -d $K --samples $gm_n -O analog
    emu_options mode=send msgs=$(gm5 13 1 1 0)30,$(gm10 13 1 1 16 12345)$(gm10 13 1 1 32 12345)$(gm10 13 1 1 64 12345)$(gm10 13 1 1 128 12345)$(gm10 13 15 1 16 12345)$(gm10 13 15 1 128 12345)$(gm5 13 2 1 0)$(gm6 1 1 12345)$(gm5 13 2 1 0)
    emu_case "18 special characters and five and six" -- -d $K --samples 7 -O analog

    # A stream's framing: what comes before a start; a data byte with no start
    # once in step; a start inside a block, which is data until five bytes.
    emu_options mode=send msgs=$S,303030$(gm13 1 1 1 0 123456)
    emu_case "data ahead of the first start" -- -d $K --samples 1
    emu_options mode=send msgs=$S,$(gm13 1 1 1 0 123456)30$(gm13 1 1 1 0 654321)
    emu_case "a data byte between blocks" --stderr --filter "$L" -- -d $K --samples 2
    emu_options mode=send msgs=$S,0131$(gm13 1 1 1 0 654321)$(gm13 1 1 1 0 123456)
    emu_case "a block cut short by a start" -- -d $K --samples 2 -O analog
    emu_options mode=send msgs=$S,$(gm5 1 1 1 0)30303030$(gm13 1 1 1 0 654321)
    emu_case "a block ended by a start past five" -- -d $K --samples 1 -O analog

    # Nothing comes, and an unplug: a meter's rate may be ten minutes, so a sample
    # limit waits.
    emu_options mode=send blocks=2
    emu_case "unplug in a capture" --oxrok-fails "the port gone" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 6 --tx-filter '/^unplug$/q' -- -d $K --samples 5
    emu_options mode=send blocks=2
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 --tx-filter '/^interrupt$/q' -- -d $K --samples 5
    emu_options mode=send blocks=1
    emu_case "silent after the scan, time" --no-output -- -d $K --time 300ms
    # The sign. C takes it to the exponent, where it is 0 for these.
    emu_options mode=send msgs=$S,$(gm13 1 1 9 0 654321)$(gm13 1 6 9 0 654321)$(gm5 1 1 9 0)$(gm6 1 9 12345)
    emu_case "2x negative" --oracle-differs "$SIGN_BUG" -- -d $K --samples 3 -O analog
    # And where it is not 0 yet: capacitance, a continuity voltage, power.
    emu_options mode=send msgs=$S,$(gm13 1 9 1 0 654321)$(gm13 1 9 9 0 654321)$(gm13 1 16 1 0 654321)$(gm13 1 16 9 0 654321)$(gm13 1 13 1 0 654321)$(gm13 1 14 1 0 654321)$(gm13 1 9 0 0 654321)$(gm13 1 9 3 0 654321)$(gm13 1 9 7 0 654321)
    emu_case "2x exponents" --oracle-differs "$EXP_BUG" -- -d $K --samples 9 -O analog
    emu_options mode=send msgs=$(gm5 13 3 8 0)30,$(gm10 13 3 11 0 12345)$(gm10 13 9 9 0 12345)
    emu_case "18 negative" --oracle-differs "$SIGN_BUG" -- -d $K --samples 2 -O analog
    emu_options mode=send msgs=$(gm5 13 3 3 0)30,$(gm10 13 3 3 0 a2345)$(gm10 13 3 3 0 12345)
    emu_case "18 overload" --oracle-differs "$OL18_BUG" -- -d $K --samples 2 -O analog

    # The bidirectional driver.
    emu_options
    emu_case "bd scan" --driver gmc-mh-2x-bd232 --keep-header -- -d $B:serialcomm=9600/8n1 --scan
    emu_case "bd show" --driver gmc-mh-2x-bd232 --keep-header -- -d $B --show
    emu_case "bd get power_off" --driver gmc-mh-2x-bd232 -- -d $B --get power_off
    emu_case "bd set power_off" --driver gmc-mh-2x-bd232 --no-output -- -d $B -c power_off=1 --set
    # Both refuse it and send nothing; what each says is its own, and a test pins it.
    emu_case "bd set power_off 0" --driver gmc-mh-2x-bd232 --expect-fail --no-output -- -d $B -c power_off=0 --set
    emu_case "bd no such node" --driver gmc-mh-2x-bd232 --keep-header --no-output -- -d gmc-mh-2x-bd232:conn=/dev/ttyACM97 --scan
    emu_case "bd no conn" --driver gmc-mh-2x-bd232 --keep-header --no-output -- -d gmc-mh-2x-bd232 --scan
    emu_options second=2
    emu_case "bd two meters" --oracle-differs "$SECOND_BUG" --driver gmc-mh-2x-bd232 --keep-header -- -d $B --scan
    emu_options asleep=1
    emu_case "bd asleep" --driver gmc-mh-2x-bd232 --keep-header -- -d $B --scan
    emu_options high=c0
    emu_case "bd high bits" --driver gmc-mh-2x-bd232 --keep-header -- -d $B --scan
    emu_options status=$(gm_status 1 1 34)
    emu_case "bd a 22M with memory" --driver gmc-mh-2x-bd232 --keep-header -- -d $B --scan
    emu_options answers=0
    emu_case "bd silent" --driver gmc-mh-2x-bd232 --keep-header --no-output -- -d $B --scan
    emu_options addr=17
    emu_case "bd address past 15" --driver gmc-mh-2x-bd232 --keep-header --stderr --filter "$L" -- -d $B --scan
    # What fails the checks, each in one answer.
    gm_frames=
    for gm_e in 1 2 3 4 5 6; do gm_frames=$gm_frames$(gm_frame 0100$(printf %02x $gm_e)00000000000000000000); done
    gm_frames=$gm_frames$(gm_frame 01273e03$(gm_status 1 1 1))
    gm_frames=${gm_frames}01273f03$(gm_status 1 1 1)00
    for gm_c in 1 2 4 5 6 7 9; do gm_frames=$gm_frames$(gm_frame 01273f0$gm_c$(gm_status 1 1 1)); done
    gm_frames=$gm_frames$(gm_frame 042b3f03$(gm_status 1 1 1))$(gm_frame 032b3f03$(gm_status 1 1 1))
    emu_options frames=-,-,-,$gm_frames
    emu_case "bd answers that fail" --driver gmc-mh-2x-bd232 --keep-header --stderr --filter "$L" -- -d $B --scan
    emu_options frames=-,-,-,01273f03
    emu_case "bd an answer cut short" --driver gmc-mh-2x-bd232 --keep-header --no-output -- -d $B --scan

    # Its capture. One request out at a time, the status every tenth, and one more
    # request after the limit.
    emu_options
    emu_case "bd samples 25 analog" --driver gmc-mh-2x-bd232 -- -d $B --samples 25 -O analog
    # Every function, then every range, in one capture.
    gm_meas=
    gm_n=0
    for gm_f in 0 1 2 3 8 10 11 12 15 17 18 19 20 21 22 23 24 25 26 28 29 30 31 32; do
        gm_meas=$gm_meas,$(gm_meas $gm_f 17 654321)
        gm_n=$((gm_n + 1))
    done
    for gm_f in 1 8 10 11 18 15 17; do
        for gm_r in 0 1 2 3 4 5 6 7; do
            gm_meas=$gm_meas,$(gm_meas $gm_f $((16 + gm_r)) 654321)
            gm_n=$((gm_n + 1))
        done
    done
    emu_options meas=${gm_meas#,}
    emu_case "bd functions and ranges" --driver gmc-mh-2x-bd232 -- -d $B --samples $gm_n -O analog
    # Where C takes the sign to an exponent that is not 0 yet.
    gm_meas=
    gm_n=0
    for gm_f in 4 5 6 7 9 13 14 16 27; do
        for gm_r in 0 1 2 3 4 5 6 7; do
            gm_meas=$gm_meas,$(gm_meas $gm_f $((16 + gm_r)) 654321)
            gm_n=$((gm_n + 1))
        done
    done
    # The sign, by tables TR and TR 2. Measuring a current goes by the second.
    for gm_one in $(gm_meas 1 25 654321) $(gm_meas 4 25 654321) $(gm_meas 4 17 654321) $(gm_meas 6 25 654321) $(gm_meas 6 17 654321) $(gm_meas 9 17 654321) $(gm_meas 9 25 654321); do
        gm_meas=$gm_meas,$gm_one
        gm_n=$((gm_n + 1))
    done
    emu_options meas=${gm_meas#,}
    emu_case "bd currents and exponents" --oracle-differs "$CURRENT_BUG" --driver gmc-mh-2x-bd232 -- -d $B --samples $gm_n -O analog
    # OL, OPEN, reserved digits; old data makes no sample.
    emu_options meas=$(gm_meas 1 17 a54321),$(gm_meas 1 17 e54321),$(gm_meas 1 17 b54321),$(gm_meas 1 1 654321),$(gm_meas 1 17 123456),$(gm_meas 1 17 111111)
    emu_case "bd digits and old data" --driver gmc-mh-2x-bd232 -- -d $B --samples 5 -O analog
    emu_options meas=$(gm_meas 1 17 d54321),$(gm_meas 1 17 123456)
    emu_case "bd a broken fuse" --oracle-differs "$FUSE_BUG" --driver gmc-mh-2x-bd232 -- -d $B --samples 2 -O analog
    emu_case "bd a broken fuse, its message" --oracle-differs "$FUSE_BUG" --driver gmc-mh-2x-bd232 --stderr --filter "$L" -- -d $B --samples 2 -O analog
    # A status changes the range's AUTO and the meter's RMS. MAN and a 22S here.
    emu_options status=$(gm_status 1 33 2)
    emu_case "bd manual range and no RMS" --driver gmc-mh-2x-bd232 -- -d $B --samples 1 -O analog
    # An error from the meter, or a checksum, costs that answer and no more.
    emu_options frames=-,-,-,-,$(gm_frame 01000500000000000000000000),$(gm_frame 01273f08$(gm_meas 1 17 123456))
    emu_case "bd error answer" --driver gmc-mh-2x-bd232 --stderr --filter "$L" -- -d $B --samples 1
    emu_case "bd error answer, the output" --driver gmc-mh-2x-bd232 -- -d $B --samples 1
    # A status of a model unknown leaves it none, which takes RMS away.
    emu_options frames=-,-,-,-,-,-,-,-,-,-,-,-,-,$(gm_frame 01273f03$(gm_status 1 1 31)),-,- meas=$(gm_meas 2 17 654321)
    emu_case "bd a status of another model" --driver gmc-mh-2x-bd232 --stderr --filter "$L" -- -d $B --samples 11
    # A function past the table's, in both places.
    emu_options frames=-,-,-,-,$(gm_frame 01273f08$(gm_meas 33 17 654321))
    emu_case "bd function out of the table" --driver gmc-mh-2x-bd232 --stderr --filter "$L" -- -d $B --samples 1
    # Time decides the rounds: answers run out so that the count does not.
    emu_options answers=5
    emu_case "bd time" --driver gmc-mh-2x-bd232 -- -d $B --time 300ms
    # A request that goes unanswered is sent again after 1 s. Its answer may be the
    # first's, so oxrok does not count it, and asks once more 2 s on.
    emu_options miss=5
    emu_case "bd a missed request" --oracle-differs "$LATE_BUG" --driver gmc-mh-2x-bd232 -- -d $B --samples 1
    emu_options answers=5
    emu_case "bd unplug" --oxrok-fails "the port gone" --driver gmc-mh-2x-bd232 --unplug model --timeout 40 --tx-filter '/^unplug$/q' -- -d $B --samples 5
    # Each status sets the model, which decides RMS for the next nine readings.
    gm_list=
    for gm_t in 1 33 2 34 3 4 5 12 14; do
        gm_list=$gm_list,$(gm_status 1 1 $gm_t)
    done
    emu_options status=$(gm_status 1 1 1),$(gm_status 1 1 1),$(gm_status 1 1 1),$(gm_status 1 1 1)$gm_list meas=$(gm_meas 2 17 654321)
    emu_case "bd each model's RMS" --driver gmc-mh-2x-bd232 -- -d $B --samples 95 -O analog
    # A request read as an answer, for address 0, reads as one.
    emu_options frames=-,-,-,-,$(gm_frame 032b3f08$(gm_meas 1 17 654321)),-
    emu_case "bd a request in the answers" --driver gmc-mh-2x-bd232 -- -d $B --samples 2
    # Bytes lost: every block from there on straddles two answers in C. Its checks
    # refuse most of them, not all: a lost digit equal to the address leaves the sum
    # good, 4.3% of streams of random digits, and it reads a wrong number with no
    # error. oxrok's refuse a digit above 0x0f as well, which brought that to 0 of
    # 84,000, and find the header. The sample after it is where they part: its time
    # limit ends C with one, and its log is oxrok's.
    emu_options cut=6:3
    emu_case "bd an answer short by three" --oracle-differs "$SHIFT_BUG" --driver gmc-mh-2x-bd232 --stderr --filter "$L" --tx-filter "$F_REQUESTS" -- -d $B --samples 2 --time 6s
    # 35 s: C has to run into it, and CI's slow sleeps stretch the scan's 3 s of
    # them past 18 s, ahead of oxrok's 10 s.
    emu_options answers=4
    emu_case "bd silent from the capture on" --oxrok-fails "no usable answer from the meter for 10 s" --oracle-bug "$SILENT_BUG" --oracle-hangs --driver gmc-mh-2x-bd232 --no-output --timeout 35 --tx-filter "$F_REQUESTS" -- -d $B --samples 1
    emu_options answers=5
    emu_case "bd interrupt" --oracle-bug "$SIGINT_BUG" --driver gmc-mh-2x-bd232 --interrupt model --timeout 40 --tx-filter '/^interrupt$/q' -- -d $B --samples 5
    # An answer after its request expired, which holds the next behind it.
    emu_options late=5:75 meas=$(gm_meas 1 17 654321),$(gm_meas 1 17 123456),$(gm_meas 1 17 111111),$(gm_meas 1 17 222222),$(gm_meas 1 17 333333)
    emu_case "bd a late answer" --oracle-differs "$LATE_BUG" --driver gmc-mh-2x-bd232 --tx-filter "$F_REQUESTS" -- -d $B --samples 2

    # As the oracle really is, with the exclusive open.
    SIGROK_CLI=$GM_STOCK_CLI
    emu_options mode=send
    emu_case "stock scan" --keep-header -- -d $K --scan
    emu_case "stock show" --oracle-bug "$LOCK_BUG" --keep-header -- -d $K --show
    emu_case "stock samples" --oracle-bug "$LOCK_BUG" -- -d $K --samples 1
    emu_options
    emu_case "stock bd samples" --oracle-bug "$LOCK_BUG" --driver gmc-mh-2x-bd232 -- -d $B --samples 1
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=gmc-mh-1x-2x-rs232:conn=$EMU_TTY
    B=gmc-mh-2x-bd232:conn=$EMU_TTY
}
