# The ZEROPLUS Logic Cube LAP-C series (emu/src/models/zeroplus-logic-cube.rs):
# a vendor USB device, 0c12:PID, with no firmware to upload.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". oxrok has the driver (src/drivers/zeroplus_logic_cube): every
# case runs both tools.
EMU_MODEL=zeroplus-logic-cube
EMU_DRIVER=zeroplus-logic-cube
EMU_FULL=zeroplus-logic-cube
EMU_GCOV_SHARED="src/usb.c"

# The model's options for the cases that follow (see fx2lafw.sh).
zp_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "zeroplus-logic-cube: the emulator refused '$*'"
}

# Claims interface 0 through usbfs as another program would, until
# zp_release (as h4032l_hold in hantek-4032l.sh).
zp_hold() {
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/zp-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/zp-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "zeroplus-logic-cube: could not claim the interface: $(cat "$TMP/zp-hold")"
            return 1
        fi
        sleep 0.1
    done
}

zp_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    Z=zeroplus-logic-cube
    # oxrok's log prefix as sigrok-cli's, so that the filters below keep
    # the driver's lines of both, and drop only the tools' own.
    P='s/^oxrok: [a-z]+: [a-z0-9_:]*zeroplus_logic_cube[a-z0-9_:]*: /zeroplus-logic-cube: /'
    # The driver's own messages on stderr, without sigrok-cli's prefix
    # and the timestamps of -l 5.
    E="$P; /zeroplus-logic-cube: /!d; s/.*zeroplus-logic-cube: //"
    # The driver's lines about the capture (-l 5): the registers it set
    # and the addresses the board reported.
    A="$P; /zeroplus-logic-cube: /!d; /(address|Status|Ramsize|Memory size|discard|samplerate regs|memory size|threshold) /!d; s/.*zeroplus-logic-cube: //"
    # The last stderr line, sigrok-cli's own.
    L='$!d'
    C='s/conn: [0-9]+\.[0-9]+/conn: BUS.ADDR/'
    # A few channels, and what a capture of them is held to: the first
    # 64 samples, the trigger marker and, through --digest, the SHA-256
    # of the whole output. B1 (bit 9) tells apart samples 512 apart,
    # such as a packet and the one before it.
    CH=A0,A1,A2,A7,B0,B1,B7
    B='1,7b; /^T:/b; d'
    # Why sigrok-cli differs or fails, with the C lines.
    NOCONN="api.c:450 lists no scan option (NO_OPTS) and api.c:187 ignores any: libsigrok's check_options() refuses conn=, so a board cannot be selected"
    RATE="zp_set_samplerate() loops on ARRAY_SIZE() (api.c:148), always true, and reads past samplerates_200[]"
    ADDR="analyzer_get_stop_address() (analyzer.c:641-642) makes an address of the failed read's error code, 0xfffffff7: at a limit of 1000 the discard it gives covers the readout, at 1500 samples go out by it"
    STALE="api.c:605-608 only warns of a short or failed bulk read and api.c:615-648 sends the buffer whole, the previous read's bytes where the new are missing"
    WAIT="analyzer_wait() (analyzer.c:411-415) has no bound and takes an error for a status: LIBUSB_ERROR_PIPE, -9, has the busy bit set"
    GONE="analyzer_wait() takes LIBUSB_ERROR_NO_DEVICE, -4, for a status with the busy bit clear, and the status and addresses read 0xfffffffc (analyzer.c:411-415, 629-655): no samples, exit 0"
    SIGINT="sigrok-cli has no SIGINT handler"

    zp_options
    emu_case "scan" --keep-header -- -d $Z --scan
    # The driver lists no scan option, so libsigrok refuses a conn=
    # (hwdriver.c check_options()) and the board cannot be selected by
    # its address or ids. oxrok takes conn= without listing it (--show
    # is as sigrok-cli's) and then names it in the device line.
    emu_case "by address" --oracle-bug "$NOCONN" --digest --filter "$B" -- -d $Z:conn=$EMU_USB -C $CH --samples 1000 -O bits
    emu_case "by ids" --oracle-bug "$NOCONN" --digest --filter "$B" -- -d $Z:conn=0c12.700e -C $CH --samples 100 -O bits
    emu_case "by ids scan" --oracle-differs "$NOCONN" --keep-header -- -d $Z:conn=0c12.700e --scan
    emu_case "by the ids of another board" --oracle-bug "$NOCONN" --expect-fail --stderr --filter "$L" -- -d $Z:conn=0c12.7009 --show
    emu_case "show" --keep-header --filter "$C" -- -d $Z --show
    emu_case "default capture" --record --digest --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits
    emu_case "default capture registers" --stderr --filter "$A" -- -l 5 -d $Z -C $CH --samples 1000 -O bits
    # At full memory: with no -t, api.c:556-557 and 591-592 make the
    # trigger offset 0 and api.c:639-640 sends the marker with no look at
    # devc->trigger. Kept: commit 4c1433d1 says the board triggers at once.
    emu_case "all channels" --digest --filter '1,40!d' -- -d $Z --samples 2048 -O hex
    # Sample counts: the memory size code and the post-trigger count
    # follow them (protocol.c set_limit_samples(), set_triggerbar()).
    for n in 2 3 4 100 2047 2048 2049 3000 16384 16385 32768; do
        emu_case "samples $n" --digest --filter "$B" -- -d $Z -C $CH --samples $n -O bits
    done
    emu_case "samples 2049 registers" --stderr --filter "$A" -- -l 5 -d $Z -C A0 --samples 2049 -O bits
    # Three samples: stop - now underflows at api.c:582 and the discard
    # that follows covers the whole readout.
    emu_case "samples 3 registers" --stderr --filter "$A" -- -l 5 -d $Z -C A0 --samples 3 -O bits
    # One sample: set_triggerbar() asks for 0 after the trigger and the
    # board reports an empty capture.
    emu_case "samples 1" --stderr --filter "$A" -- -l 5 -d $Z -C A0 --samples 1 -O bits
    # No limit: libsigrok refuses a limit of 0 (hwdriver.c
    # check_key()), and sigrok-cli runs no capture without a limit. The
    # filter maps the two tools' prefixes, hwdriver's and the driver's.
    emu_case "samples 0 refused" --expect-fail --stderr --filter "s/^sr: hwdriver: //; $P; s/^zeroplus-logic-cube: //" -- -d $Z -C $CH --samples 0 -O bits
    # More than the board holds: sigrok-cli refuses.
    emu_case "samples 40000 refused" --expect-fail --stderr -- -d $Z -C $CH --samples 40000 -O bits
    emu_case "time 1 ms" --digest --filter "$B" -- -d $Z -C $CH --time 1 -O bits
    # A limit set through -c is not checked by sigrok-cli: the driver
    # clamps it to the board's 32768 (protocol.c:64-65), then --time
    # sets its own.
    emu_case "limit_samples 40000 clamped" --stderr --filter "$P; /memory size/!d; s/.*zeroplus-logic-cube: //" -- -l 4 -d $Z -C A0 -c limit_samples=40000 --time 1 -O bits
    # Every rate of the list: each line of __analyzer_set_freq()'s table.
    for r in 100 500 1k 5k 25k 50k 100k 200k 400k 800k 1M 10M 25M 50M 80M 100M; do
        emu_case "samplerate $r" --stderr --filter "$A" -- -l 5 -d $Z -C A0 -c samplerate=$r --samples 100 -O bits
    done
    # In samplerates_200[] but over the board's 100 MHz.
    for r in 150M 200M; do
        emu_case "samplerate $r refused" --expect-fail --stderr --filter "$E" -- -d $Z -C A0 -c samplerate=$r --samples 100 -O bits
    done
    # zp_set_samplerate() looks a rate up with a loop whose condition is
    # ARRAY_SIZE(samplerates_200), always true (api.c:148): a rate not in
    # the list reads past the array until a crash or a match, which depends
    # on the rodata of the oracle's build, as do these cases. The core dump
    # line is what pins the crash in the .oracle (timeout prints it, the
    # kernel handing dumps to systemd-coredump on these hosts). oxrok
    # refuses the rate.
    for r in 3M 123456789; do
        emu_case "samplerate $r crashes" --oracle-bug "$RATE" --expect-fail --stderr --filter "$E" -- -d $Z -C A0 -c samplerate=$r --samples 100 -O bits
    done
    # A value that does lie in that rodata: accepted, exit 0, though no
    # table has it. __analyzer_set_freq() returns -1, unchecked
    # (analyzer.c:326-327), and the capture writes no rate register: the
    # board stays on the 1 MHz of the open while the output says 46 Hz,
    # the line the .oracle keeps. oxrok refuses the rate.
    emu_case "samplerate 46 accepted" --oracle-differs "$RATE" --expect-fail --stderr --keep-header --filter "$P; /zeroplus-logic-cube: |^Acquisition with/!d; s/.*zeroplus-logic-cube: //" -- -d $Z -C A0 -c samplerate=46 --samples 100 -O bits
    # Refused by check_key() in libsigrok, by the driver in oxrok.
    emu_case "captureratio 101 refused" --expect-fail --stderr --filter '/Capture ratio/!d; s/.*: //' -- -d $Z -C A0 -c captureratio=101 --samples 100 -O bits
    # Triggers on the counter the model plays: channel n is bit n.
    emu_case "trigger A0 high" --digest --filter "$B" -- -d $Z -C $CH -t A0=1 --samples 1000 -O bits
    emu_case "trigger A1 low ratio 50" --digest --filter "$B" -- -d $Z -C $CH -t A1=0 -c captureratio=50 --samples 1000 -O bits
    emu_case "trigger A2 rising ratio 30" --record --digest --filter "$B" -- -d $Z -C $CH -t A2=r -c captureratio=30 --samples 1000 -O bits
    emu_case "trigger A7 falling ratio 10" --digest --filter "$B" -- -d $Z -C $CH -t A7=f -c captureratio=10 --samples 1000 -O bits
    emu_case "trigger A3 edge ratio 50" --digest --filter "$B" -- -d $Z -C $CH,A3 -t A3=e -c captureratio=50 --samples 1000 -O bits
    emu_case "trigger levels" --digest --filter "$B" -- -d $Z -C $CH -t A0=1,A1=0,A2=1,B0=1 -c captureratio=20 --samples 1000 -O bits
    emu_case "trigger levels registers" --stderr --filter "$A" -- -l 5 -d $Z -C $CH -t A0=1,A1=0,A2=1,B0=1 -c captureratio=20 --samples 1000 -O bits
    # The trigger late: the memory wraps while the board waits.
    emu_case "trigger B7 rising wraps" --digest --filter "$B" -- -d $Z -C $CH -t B7=r -c captureratio=50 --samples 1000 -O bits
    emu_case "trigger B7 rising registers" --stderr --filter "$A" -- -l 5 -d $Z -C $CH -t B7=r -c captureratio=50 --samples 1000 -O bits
    # The capture fills the memory exactly: the now address is 0.
    emu_case "trigger fills memory" --stderr --filter "$A" -- -l 5 -d $Z -C A0 -t A0=1 --samples 2048 -O bits
    emu_case "trigger fills memory output" --digest --filter "$B" -- -d $Z -C $CH -t A0=1 --samples 2048 -O bits
    # A ratio of 100: nothing after the trigger.
    emu_case "trigger ratio 100" --digest --filter "$B" -- -d $Z -C $CH -t A2=r -c captureratio=100 --samples 1000 -O bits
    emu_case "trigger ratio 100 registers" --stderr --filter "$A" -- -l 5 -d $Z -C A2 -t A2=r -c captureratio=100 --samples 1000 -O bits
    # Nothing left after the trigger at limits 1 and 2: protocol.c:113-118
    # wraps the count to -1(0xffffffff) and runs on, and so must the port.
    for n in 1 2; do
        emu_case "trigger ratio 100 samples $n" --stderr --filter "$A" -- -l 5 -d $Z -C A0 -t A0=1 -c captureratio=100 --samples $n -O bits
    done
    # Two edges: the board has one edge register, the driver keeps the
    # last.
    emu_case "two edges" --stderr --filter "$A" -- -l 5 -d $Z -C $CH -t A0=r,A1=f --samples 1000 -O bits
    emu_case "two edges output" --digest --filter "$B" -- -d $Z -C $CH -t A0=r,A1=f --samples 1000 -O bits
    # The same registers, and oxrok says so where the C is silent.
    emu_case "two edges warned" --oracle-differs "analyzer.c:537-545 replaces the first edge silently" --stderr --filter "$P; /one edge/!d; s/.*zeroplus-logic-cube: //" -- -d $Z -C $CH -t A0=r,A1=f --samples 1000 -O bits
    # A trigger on a channel left out: sigrok-cli refuses it, so the
    # driver's check (analyzer.c:526-528) is not reached.
    emu_case "trigger on disabled channel refused" --expect-fail --stderr -- -d $Z -C A1,A2 -t A0=1 --samples 1000 -O bits
    # Thresholds: (int)round(-9.1 * V + 62.6) in the four level registers.
    # Out of the listed range the driver clamps at 6 V and -6 V; a low
    # and high pair is taken as their mean.
    for t in 1.5:1.5-1.5 3.3:3.3-3.3 minus6:-6--6 6:6-6 0:0-0 range:0-3.3 9:9-9 minus9:-9--9; do
        emu_case "threshold ${t%%:*}" --stderr --filter "$A" -- -l 5 -d $Z -C A0 -c voltage_threshold=${t#*:} --samples 100 -O bits
    done
    emu_case "external clock" --stderr --filter "$A" -- -l 5 -d $Z -C A0 -c external_clock=1 --samples 100 -O bits
    emu_case "external clock falling" --digest --filter "$B" -- -d $Z -C $CH -c external_clock=1:clock_edge=falling --samples 100 -O bits
    emu_case "clock edge falling internal" --digest --filter "$B" -- -d $Z -C $CH -c clock_edge=falling --samples 100 -O bits
    # The last line, without the reason oxrok adds to it.
    emu_case "set clock_edge up" --expect-fail --stderr --filter "$L; s/^(Failed to set device option .clock_edge.: invalid argument).*/\\1./" -- -d $Z -c clock_edge=up --samples 100
    for k in samplerate captureratio voltage_threshold external_clock clock_edge; do
        emu_case "get $k" -- -d $Z --get $k
    done
    emu_case "get limit_samples" --expect-fail --stderr --filter "$L" -- -d $Z --get limit_samples
    emu_case "get samplerate set" -- -d $Z -c samplerate=25M --get samplerate
    emu_case "get captureratio set" -- -d $Z -c captureratio=40 --get captureratio
    emu_case "get voltage_threshold set" -- -d $Z -c voltage_threshold=2.5-2.5 --get voltage_threshold
    emu_case "get clock_edge set" -- -d $Z -c external_clock=1:clock_edge=falling --get clock_edge
    # A level high and low: the C ORs the bits (analyzer.c:532-535), and
    # the model takes pair 11 as both, which never fires. Neither tool
    # refuses it, as nothing says what a board makes of it; both wait, so
    # libsigrok alone can run to its timeout here. oxrok is held below, by
    # the unplug that ends its wait.
    emu_case "two levels on one channel hangs" --libsigrok --hangs --timeout 2 --stderr --no-output -- -d $Z -C A0 -t A0=0,A0=1 --samples 100 -O bits
    # Polls are not paced, so an interrupt there would land on no known
    # count; an unplug after the fifth status read does. The C goes on
    # with its error code for a status; oxrok warned first, sent the same
    # registers, and ends.
    emu_case "two levels warned" --oracle-differs "$GONE" --stderr --filter "$P; /high and low|Lost the board/!d; s/.*zeroplus-logic-cube: //" --unplug in=0:5 -- -d $Z -C A0 -t A0=0,A0=1 --samples 100 -O bits
    # What sigrok-cli does there: no samples, exit 0, the same transactions.
    emu_case "two levels unplugged output" --no-output --unplug in=0:5 -- -d $Z -C A0 -t A0=0,A0=1 --samples 100 -O bits

    # Other boards.
    zp_options pid=0x7009
    emu_case "LAP-C(16064) show" --keep-header --filter "$C" -- -d $Z --show
    emu_case "LAP-C(16064) samples 65536" --digest --filter "$B" -- -d $Z -C $CH --samples 65536 -O bits
    zp_options pid=0x700b
    emu_case "LAP-C(32128) show" --keep-header --filter "$C" -- -d $Z --show
    emu_case "LAP-C(32128) all channels" --digest --filter '1,40!d' -- -d $Z --samples 2048 -O hex
    emu_case "LAP-C(32128) trigger D0 rising" --stderr --filter "$A" -- -l 5 -d $Z -C D0 -t D0=r -c captureratio=50 --samples 1000 -O bits
    emu_case "LAP-C(32128) trigger C1 and D7" --digest --filter "$B" -- -d $Z -C C1,D7 -t C1=1,D7=0 -c captureratio=50 --samples 1000 -O bits
    # A board of 200 MHz takes the two rates the others refuse.
    zp_options pid=0x7002
    for r in 150M 200M; do
        emu_case "LAP-16128U samplerate $r" --stderr --filter "$A" -- -l 5 -d $Z -C A0 -c samplerate=$r --samples 100 -O bits
    done
    zp_options pid=0x7100
    emu_case "AKIP-9101 show" --keep-header --filter "$C" -- -d $Z --show
    emu_case "AKIP-9101 samples 262144" --digest --filter '1,7b; /^T:/b; d' -- -d $Z -C A0 --samples 262144 -O bits
    # With a serial number: the scan reads it.
    zp_options serial=ZP1234
    emu_case "serial scan" --keep-header -- -d $Z --scan
    # Its read refused: the board is skipped (and the handle leaks).
    zp_options serial=ZP1234 stall_string=1
    emu_case "serial refused" --stderr --filter "$E" -- -l 4 -d $Z --scan

    # A board an earlier session configured: the open does not configure
    # it again (status not 0).
    zp_options warm=1
    emu_case "warm open" --stderr --filter "$A" -- -l 5 -d $Z -C A0 --samples 100 -O bits
    emu_case "warm capture" --digest --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits
    # Busy for more polls, and for none.
    zp_options fill=6
    emu_case "fill 6" --digest --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits
    zp_options fill=0
    emu_case "fill 0" --digest --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits

    # Another program holds the interface.
    zp_options
    if zp_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$E" -- -d $Z --samples 100
        zp_release
    fi

    # Faults. The open's status read refused: the open does not
    # configure the board (the error is not 0).
    zp_options stall=rdata:0x21@1
    emu_case "fault open status read" --stderr --filter "$E" -- -d $Z -C A0 --samples 100 -O bits
    zp_options stall=addr:0x21@1
    emu_case "fault open status select" --stderr --filter "$E" -- -d $Z -C A0 --samples 100 -O bits
    # A register write refused in the open's configure.
    zp_options stall=wdata@1
    emu_case "fault first write" --stderr --filter "$E" -- -d $Z -C A0 --samples 100 -O bits
    # The capture's frequency registers refused, one each: the rest of
    # the rate is not sent and the board keeps the open's 1 MHz.
    for r in 0x30 0x31 0x32 0x34; do
        zp_options stall=wdata:$r@2
        emu_case "fault rate register $r" --stderr --filter "$E" -- -d $Z -C A0 -c samplerate=25M --samples 100 -O bits
    done
    zp_options stall=addr:0x30@2
    emu_case "fault rate select" --stderr --filter "$E" -- -d $Z -C A0 -c samplerate=25M --samples 100 -O bits
    # A status poll refused: its error has the busy bit set, so the
    # driver polls on.
    zp_options stall=rdata:0x21@2
    emu_case "fault poll" --digest --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits
    zp_options stall=rdata:0x21@2
    emu_case "fault poll stderr" --stderr --filter "$E" -- -d $Z -C A0 --samples 100 -O bits
    # An address read refused: the C's Stop address is 0xfffffff7 (Need to
    # discard 1089 at a limit of 1500) and it delivers samples by that,
    # the output case's .oracle; oxrok says so, its .out, and delivers none.
    zp_options stall=rdata:0x9b@1
    emu_case "fault stop address read" --oracle-differs "$ADDR" --stderr --filter "$E" -- -d $Z -C A0 --samples 1500 -O bits
    zp_options stall=rdata:0x9b@1
    emu_case "fault stop address read output" --oracle-differs "$ADDR" --digest --filter "$B" -- -d $Z -C $CH --samples 1500 -O bits
    # The bulk read request refused: the bulk read that follows times
    # out (5 s) and the C uses its buffer all the same. oxrok ends the
    # capture with the samples of the reads that completed, no read after
    # that one: the .oracle holds the C's samples, sent again, and oxrok's
    # are those of a clean capture up to the fault. Each stderr case's .oracle is empty (a
    # stdout is all --oracle-differs keeps), its .out what oxrok says.
    zp_options stall=bulkreq@3
    emu_case "fault bulk request" --oracle-differs "$STALE" --stderr --filter "$E" -- -d $Z -C A0 --samples 1000 -O bits
    zp_options stall=bulkreq@3
    emu_case "fault bulk request output" --oracle-differs "$STALE" --no-output --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits
    # A bulk read refused, and one cut short.
    zp_options stall=bulk@3
    emu_case "fault bulk stall" --oracle-differs "$STALE" --stderr --filter "$E" -- -d $Z -C A0 --samples 1000 -O bits
    zp_options stall=bulk@3
    emu_case "fault bulk stall output" --oracle-differs "$STALE" --no-output --filter "$B" -- -d $Z -C $CH --samples 1000 -O bits
    zp_options short=4
    emu_case "fault short read" --oracle-differs "$STALE" --stderr --filter "$E" -- -d $Z -C A0 --samples 1000 -O bits
    # The whole output: 740 samples, the first 740 of "default capture".
    zp_options short=4
    emu_case "fault short read output" --oracle-differs "$STALE" -- -d $Z -C $CH --samples 1000 -O bits
    # A read after the first delivered ones refused, the stop on its way:
    # the signal comes with the request, so the read that follows has
    # begun and its 5 s (the model NAKs it) tell the stop on no timing.
    # oxrok ends the readout and sends the reset of dev_acquisition_stop().
    zp_options stall=bulkreq@4
    emu_case "interrupt during the readout" --oracle-bug "$SIGINT" --interrupt out=0:236 -- -d $Z -C $CH --samples 1000 -O bits

    # What the C never ends: every status read refused (an error with
    # the busy bit set). oxrok gives up after protocol::MAX_FAILED_POLLS,
    # paced to about 1.6 s.
    zp_options stall=rdata:0x21
    emu_case "fault every status read hangs" --oracle-bug "$WAIT" --oracle-hangs --timeout 5 --stderr --filter "$E" -- -d $Z -C A0 --samples 100 -O bits

    # Unplugged while the driver polls (after control read #2, the
    # first poll), and in the middle of the data (after bulk IN #3).
    zp_options
    emu_case "unplug while polling" --oracle-differs "$GONE" --stderr --filter "$E" --unplug in=0:2 -- -d $Z -C A0 --samples 1000 -O bits
    # No samples and exit 0 under both tools; the C's garbage is its log.
    emu_case "unplug while polling output" --no-output --unplug in=0:2 -- -d $Z -C A0 --samples 1000 -O bits
    emu_case "unplug mid data" --oracle-differs "$STALE" --stderr --filter "$E" --unplug in=1:3 -- -d $Z -C A0 --samples 1000 -O bits
    emu_case "unplug mid data output" --oracle-differs "$STALE" --digest --filter "$B" --unplug in=1:3 -- -d $Z -C $CH --samples 1000 -O bits
    zp_options
}
