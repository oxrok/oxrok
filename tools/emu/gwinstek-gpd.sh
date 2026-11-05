# A GW Instek GPD-2303S or GPD-3303S (emu/src/models/gwinstek-gpd.rs) on the
# CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md,
# "Writing a serial model".
EMU_MODEL=gwinstek-gpd
EMU_CDC=1
EMU_DRIVER=gwinstek-gpd
EMU_GCOV=gwinstek-gpd
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

# api.c's scan returns with the port open (no serial_close before
# std_scan_complete), and dev_open opens it again, which libserialport's
# exclusive open refuses (serialport.c:589 flock, :599 TIOCEXCL): on this
# oracle stock sigrok-cli runs nothing but --scan with this driver. So every
# full case below runs sigrok-cli with locking disabled: a preload makes flock()
# and the one ioctl TIOCEXCL no-ops, and the C driver itself is unmodified. It
# is set on the exec of this model's sigrok-cli only, by a wrapper that
# SIGROK_CLI names for the length of emu_cases, and neither oxrok nor the
# harness runs under it. The "stock" cases at the end run without it and pin
# the failure with its stderr.
gpd_openable() {
    GPD_STOCK_CLI=$SIGROK_CLI
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
        '}' >"$TMP/gpd-openable.c"
    cc -shared -fPIC -o "$TMP/gpd-openable.so" "$TMP/gpd-openable.c" -ldl 2>"$TMP/gpd-openable.err" \
        || { emu_fail "$EMU_MODEL: could not build the preload (a C compiler?)"; return; }
    printf '%s\n' '#!/bin/sh' \
        "LD_PRELOAD=$TMP/gpd-openable.so\${LD_PRELOAD:+:\$LD_PRELOAD} exec $(command -v "$SIGROK_CLI") \"\$@\"" \
        >"$TMP/gpd-sigrok-cli"
    chmod +x "$TMP/gpd-sigrok-cli"
    SIGROK_CLI=$TMP/gpd-sigrok-cli
}

emu_cases() {
    gpd_openable
    emu_conns
    S2303=$(emu_hex 'GW INSTEK,GPD-2303S,SN:ER915277,V2.10')
    S3303=$(emu_hex 'GW INSTEK,GPD-3303S,SN:ER000001,V1.20')
    BAD4303=$(emu_hex 'GW INSTEK,GPD-4303S,SN:1,V1.00')

    # A scan: *IDN?, STATUS?, then ISETn? VSETn? IOUTn? VOUTn? per channel, and
    # no setting, which setters=0 would flag.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$S3303 vmax=32 imax=3.2
    emu_case "scan GPD-3303S" --keep-header -- -d $K --scan
    # The line ends: the old firmware's CR and the new one's CR LF.
    emu_options setters=0 eol=cr
    emu_case "scan with CR" --keep-header -- -d $K --scan
    emu_options setters=0 eol=lf
    emu_case "scan with LF" --keep-header -- -d $K --scan
    emu_options setters=0 chunk=1
    emu_case "scan one byte per transfer" --keep-header -- -d $K --scan
    # The rates api.c tries: nothing passes until the line fits.
    emu_options setters=0 uart=57600/8n1
    emu_case "scan at 57600" --keep-header -- -d $K --scan
    emu_options setters=0 uart=115200/8n1
    emu_case "scan at 115200" --keep-header -- -d $K --scan
    emu_options setters=0 uart=19200/8n1
    emu_case "scan at no rate it tries" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 uart=57600/8n1
    emu_case "serialcomm fits" --keep-header -- -d $K:serialcomm=57600/8n1 --scan
    emu_case "serialcomm does not fit" --keep-header --no-output -- -d $K:serialcomm=9600/8n1 --scan
    emu_options setters=0
    emu_case "serialcomm no such thing" --keep-header --no-output -- -d $K:serialcomm=fast --scan
    emu_case "no such node" --keep-header --no-output -- -d gwinstek-gpd:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d gwinstek-gpd --scan
    # api.c's comment: what lay in the buffer spoils the first *IDN?.
    emu_options setters=0 junk=VSET1:1
    emu_case "scan junk in the buffer" --keep-header -- -d $K --scan
    # The second form of the status, three lines.
    emu_options setters=0 status-old=1 eol=cr
    emu_case "scan old status" --keep-header -- -d $K --scan
    emu_case "get enabled old status" -- -d $K --get enabled
    emu_options setters=0 status-old=1 output=1 eol=cr
    emu_case "get enabled on old status" -- -d $K --get enabled
    emu_options setters=0 status=xyz
    emu_case "scan status garbage" --keep-header --no-output -- -d $K --scan
    # Status digits may have spaces between them, and more may follow.
    emu_options setters=0 status=1101
    emu_case "scan status too short" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 status=110110101234
    emu_case "scan status too long" --keep-header -- -d $K --scan
    emu_options setters=0 status=11011210
    emu_case "get enabled digit 2" -- -d $K --get enabled

    # IDs. An unsupported one: one no-match path, then the rates after it.
    emu_options setters=0 idn=$BAD4303
    emu_case "scan GPD-4303S" --stderr -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'GW INSTEK,GPD-2303S')
    emu_case "scan ID without a serial" --stderr -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'ACME,GPD-2303S,SN:1,V1')
    emu_case "scan another vendor" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'gw instek,GPD-2303S,SN:1,V1')
    emu_case "scan in lower case" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=none
    emu_case "scan no answer" --keep-header --no-output -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'GW INSTEK,GPD-3303S,x,SN:1,SN:2,V1')
    emu_case "scan the regex is greedy" --stderr -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'GW INSTEK,GPD-3303S,SN:1,V')
    emu_case "scan an empty version" --stderr -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'GW INSTEK,GPD-2303S,SN:1,V1 extra')
    emu_case "scan text after the version" --keep-header -- -d $K --scan
    emu_options setters=0 idn=$(emu_hex 'GW INSTEK,GPD-2303S,SN:1,V1') idn-eol=0
    emu_case "scan ID with no line end" --keep-header --no-output -- -d $K --scan

    # Answers gone wrong in the scan.
    emu_options setters=0 iset1=x
    emu_case "scan ISET garbage" --stderr -- -d $K --scan
    emu_options setters=0 vset2=x
    emu_case "scan VSET garbage" --stderr -- -d $K --scan
    emu_options setters=0 iout2=x
    emu_case "scan IOUT garbage" --stderr -- -d $K --scan
    emu_options setters=0 vout2=x
    emu_case "scan VOUT garbage" --stderr -- -d $K --scan
    # The buffer keeps the last answer when a read fails.
    emu_options setters=0 answers=1
    emu_case "scan status silent" --stderr -- -d $K --scan
    emu_options setters=0 answers=2
    emu_case "scan ISET silent" -- -d $K -g CH1 --get current_limit
    emu_options setters=0 answers=3
    emu_case "scan VSET silent" -- -d $K -g CH1 --get voltage_target
    emu_options setters=0 iout2=~
    emu_case "scan IOUT silent keeps VSET" -- -d $K -g CH2 --get current
    emu_options setters=0 vout1=~
    emu_case "scan VOUT silent keeps IOUT" -- -d $K -g CH1 --get voltage
    emu_options setters=0 iset1=7.5~
    emu_case "scan a short answer over the old one" -- -d $K -g CH1 --get current_limit
    emu_options setters=0 iset1=nan
    emu_case "scan nan" -- -d $K -g CH1 --get current_limit
    emu_options setters=0 iset1=-inf
    emu_case "scan inf" -- -d $K -g CH1 --get current_limit
    emu_options setters=0 iset1=0x10
    emu_case "scan hexadecimal" -- -d $K -g CH1 --get current_limit
    emu_options setters=0 iset1=1e2A
    emu_case "scan exponent" -- -d $K -g CH1 --get current_limit

    # --show and --get: all cached from the scan, nothing on the wire.
    emu_options setters=0
    emu_case "show" --keep-header -- -d $K --show
    emu_case "show CH1" --keep-header -- -d $K -g CH1 --show
    emu_case "show CH2" --keep-header -- -d $K -g CH2 --show
    emu_options setters=0 idn=$S3303 vmax=32 imax=3.2
    emu_case "show 3303" --keep-header -- -d $K --show
    emu_case "show CH2 3303" --keep-header -- -d $K -g CH2 --show
    emu_options setters=0 output=1 load=10 v1=5.000 i1=2.000 v2=3.300 i2=0.100
    for key in enabled channel_config limit_samples limit_time; do
        emu_case "get $key" -- -d $K --get $key
    done
    for g in CH1 CH2; do
        for key in voltage voltage_target current current_limit; do
            emu_case "get $g $key" -- -d $K -g $g --get $key
        done
    done
    emu_case "get voltage with no group" --expect-fail --stderr -- -d $K --get voltage
    emu_case "get enabled on a group" --expect-fail --stderr -- -d $K -g CH1 --get enabled
    emu_case "get conn" --expect-fail --stderr -- -d $K --get conn

    # A setting: a line with no answer, and C's cache takes it.
    emu_options
    emu_case "set enabled on" --no-output -- -d $K -c enabled=on --set
    emu_case "set enabled off" --no-output -- -d $K -c enabled=off --set
    emu_case "set then get enabled" -- -d $K -c enabled=on --get enabled
    emu_case "set CH1 voltage" --no-output -- -d $K -g CH1 -c voltage_target=5.5 --set
    emu_case "set CH2 voltage" --no-output -- -d $K -g CH2 -c voltage_target=3.3 --set
    emu_case "set CH1 current" --no-output -- -d $K -g CH1 -c current_limit=0.25 --set
    emu_case "set CH2 current" --no-output -- -d $K -g CH2 -c current_limit=1.5 --set
    emu_case "set voltage max" --no-output -- -d $K -g CH1 -c voltage_target=30 --set
    emu_case "set voltage min" --no-output -- -d $K -g CH1 -c voltage_target=0 --set
    emu_case "set current max" --no-output -- -d $K -g CH1 -c current_limit=3 --set
    emu_case "set voltage rounds" --no-output -- -d $K -g CH1 -c voltage_target=1.2345 --set
    emu_case "set voltage rounds in single precision" --no-output -- -d $K -g CH1 -c voltage_target=0.1235 --set
    emu_case "set voltage ten" --no-output -- -d $K -g CH1 -c voltage_target=10 --set
    emu_case "set then get voltage" -- -d $K -g CH1 -c voltage_target=3.3 --get voltage_target
    emu_case "set then get current" -- -d $K -g CH2 -c current_limit=0.1 --get current_limit
    emu_case "set voltage too high" --stderr --expect-fail -- -d $K -g CH1 -c voltage_target=30.01 --set
    emu_case "set voltage negative" --stderr --expect-fail -- -d $K -g CH1 -c voltage_target=-1 --set
    emu_case "set current too high" --stderr --expect-fail -- -d $K -g CH1 -c current_limit=3.01 --set
    emu_case "set voltage nan" --no-output -- -d $K -g CH1 -c voltage_target=nan --set
    emu_case "set current nan" --no-output -- -d $K -g CH2 -c current_limit=nan --set
    emu_case "set voltage -0" --no-output -- -d $K -g CH1 -c voltage_target=-0 --set
    emu_case "set voltage with no group" --stderr --expect-fail -- -d $K -c voltage_target=1 --set
    emu_case "set enabled on a group" --stderr --expect-fail -- -d $K -g CH1 -c enabled=on --set
    emu_case "set channel_config" --stderr --expect-fail -- -d $K -c channel_config=Independent --set
    emu_case "set several in hash order" --no-output -- -d $K -g CH1 -c current_limit=0.5:voltage_target=3.3 --set
    emu_case "set the output and a voltage" --no-output -- -d $K -g CH1 -c voltage_target=3.3 -c channel_group=global:enabled=on --set
    emu_options idn=$S3303 vmax=32 imax=3.2
    emu_case "set 3303 voltage max" --no-output -- -d $K -g CH1 -c voltage_target=32 --set
    emu_case "set 3303 voltage too high" --stderr --expect-fail -- -d $K -g CH1 -c voltage_target=32.1 --set
    emu_case "set 3303 current 3.1" --no-output -- -d $K -g CH2 -c current_limit=3.1 --set
    emu_case "set 3303 current max" --stderr --expect-fail -- -d $K -g CH2 -c current_limit=3.2 --set
    emu_options

    # A capture: every 100 ms the four queries at once, a sample per round.
    emu_options setters=0 output=1 load=10 v1=5.000 i1=2.000 v2=3.300 i2=0.100
    emu_case "samples 1" -- -d $K --samples 1
    emu_case "samples 2" -- -d $K --samples 2
    emu_case "samples 3 analog" -- -d $K --samples 3 -O analog
    emu_case "channel CH1 only" -- -d $K -C CH1 --samples 2
    emu_case "channel CH2 only" -- -d $K -C CH2 --samples 2
    emu_options setters=0 output=1 eol=cr
    emu_case "samples with CR" -- -d $K --samples 2
    emu_options setters=0 output=1 eol=lf
    emu_case "samples with LF" -- -d $K --samples 2
    emu_options setters=0 output=1 in-stall=1
    emu_case "a stalled read" -- -d $K --samples 2
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header -- -d $K --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $K --scan
    # Readings of every shape sscanf takes.
    emu_options setters=0 output=1 vout1=12.340V,1,1.5e1V,-2V,+.5V iout1=0.125A,1.0A,nan,inf,0x10
    emu_case "readings in turn" -- -d $K --samples 5
    # A time limit: how many rounds depends on the clock, so both are kept up
    # to the first round's queries.
    emu_options setters=0 output=1
    emu_case "time" --filter '5,$d' --tx-filter '/^bulk_out 02 494f5554313f0a564f5554313f0a494f5554323f0a564f5554323f0a/{x;s/^/x/;/^xx/{x;q};x}' -- -d $K --time 500ms
    emu_case "config limit_samples 0" --stderr --expect-fail -- -d $K -c limit_samples=0 --samples 1
    # The output goes on ahead of the capture, set points first.
    emu_options output=0
    emu_case "capture with the output on" -- -d $K -c enabled=on --samples 1
    emu_case "capture with a setting" -- -d $K -g CH1 -c voltage_target=4 --samples 1
    # Data no one asked for, between two rounds.
    emu_options setters=0 output=1 chatter=hello
    emu_case "chatter" -- -d $K --samples 2
    # A line longer than protocol.c's 50 byte buffer in a capture, and than
    # api.c's 100 in a scan. Each list's first answer goes to the scan.
    emu_options setters=0 output=1 iout1=0.000A,0123456789012345678901234567890123456789012345678901234567890123456789
    emu_case "a reading too long" --timeout 3 -- -d $K --samples 1
    emu_options setters=0 output=1 idn=$(emu_hex 'GW INSTEK,GPD-2303S,SN:01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890,V1')
    emu_case "an ID too long" --keep-header --no-output -- -d $K --scan

    # An invalid reading returns before the limit's check with reply_pending
    # left set, so no query follows and the C never ends; with CR LF the "\n"
    # left over makes a read fail with the buffer's old bytes, which it parses.
    emu_options setters=0 output=1 eol=lf iout1=0.000A,x
    emu_case "reading garbage" --stderr --hangs --timeout 2 -- -d $K --samples 1
    emu_options setters=0 output=1 iout1=0.000A,x
    emu_case "reading garbage with CR LF" --stderr --timeout 3 -- -d $K --samples 1
    emu_options setters=0 output=1 eol=lf vout1=12.000V,x
    emu_case "reading garbage after a good one" --stderr --hangs --timeout 2 -- -d $K --samples 1
    emu_options setters=0 output=1 eol=lf vout2=05.000V,~
    emu_case "a reading missing" --stderr --hangs --timeout 2 -- -d $K --samples 1
    emu_options setters=0 output=1 vout2=05.000V,~
    emu_case "a reading missing with CR LF" --timeout 3 -- -d $K --samples 1
    emu_options setters=0 output=1 answers=10
    emu_case "silent from the capture on" --no-output --hangs --timeout 2 -- -d $K --samples 1
    # A time limit does end those: the timer's branch checks it.
    emu_case "silent from the capture on, time" --no-output --tx-filter '/^bulk_out 02 494f5554313f0a564f5554313f0a/q' -- -d $K --time 300ms
    # goto error frees match_info and regex (api.c:270-273), and no path clears
    # them for the next rate: a later error path frees them again. Here it
    # corrupts the heap some runs ("malloc(): smallbin double linked list
    # corrupted", SIGABRT) and not others, so only the first rate is compared.
    emu_options setters=0 vout1=x
    emu_case "scan VOUT1 garbage" --stderr --expect-fail --filter '2,$d' \
        --tx-filter '/^bulk_in 82 [0-9]+ -> 780d0a$/q' -- -d $K --samples 1

    emu_options setters=0 output=1 answers=10
    emu_case "unplug in a capture" --no-output --unplug model --hangs --timeout 4 --tx-filter '/^unplug$/q' -- -d $K --samples 1
    emu_options setters=0 answers=4
    emu_case "unplug in the scan" --stderr --unplug model --timeout 30 -- -d $K --scan
    # As the oracle really is, with the exclusive open.
    SIGROK_CLI=$GPD_STOCK_CLI
    emu_options setters=0
    emu_case "stock scan" --keep-header -- -d $K --scan
    emu_case "stock show" --stderr --expect-fail -- -d $K --show
    emu_case "stock get" --stderr --expect-fail -- -d $K --get enabled
    emu_case "stock samples" --stderr --expect-fail -- -d $K --samples 1
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    K=gwinstek-gpd:conn=$EMU_TTY
}
