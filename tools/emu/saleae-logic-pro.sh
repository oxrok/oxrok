# The Saleae Logic Pro 16 (emu/src/models/saleae-logic-pro.rs): an FX3
# that takes its firmware and renumerates, then an FPGA bitstream and
# an authentication through LFSR-scrambled commands on endpoint 1.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model".
EMU_MODEL=saleae-logic-pro
EMU_DRIVER=saleae-logic-pro
EMU_GCOV_SHARED="src/usb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=saleae-logic-pro

# The FX3 firmware stand-in: the driver parses saleae-logicpro16-fx3.fw
# as parts (size LE16, address LE32, final flag, data), so the plain
# stand-in of tools/emu-firmware.sh would not upload. These are its
# SHA-256 blocks (10000 bytes) in parts of 4096 from 0x40003000, then
# the empty final part that starts it: fx3_image() of the model.
# logicpro_fx3_standin [big|truncated|nofinal|unflagged|missing] writes a broken
# one instead, for upload_firmware()'s checks of the file; the cases
# that use one write the good one back after them.
logicpro_fx3_standin() {
    python3 -I - "$SIGROK_FIRMWARE_DIR/saleae-logicpro16-fx3.fw" "${1:-good}" <<'PY'
import hashlib, os, struct, sys
name, size, base = "saleae-logicpro16-fx3.fw", 10000, 0x40003000
dest, how = sys.argv[1], sys.argv[2]
if how == "missing":
    if os.path.exists(dest):
        os.remove(dest)
    sys.exit(0)
payload = b"".join(hashlib.sha256(f"{name}:{i}".encode()).digest()
                   for i in range((size + 31) // 32))[:size]
out = b""
for i in range(0, size, 4096):
    part = payload[i:i + 4096]
    # A first part that claims 4097 bytes, over FW_MAX_PART_SIZE.
    n = 4097 if how == "big" and i == 0 else len(part)
    out += struct.pack("<HIB", n, base + i, 0) + part
if how == "truncated":
    out = out[:7 + 4096 + 7 + 100]
elif how != "nofinal":
    out += struct.pack("<HIB", 0, base, 0 if how == "unflagged" else 1)
try:
    with open(dest, "rb") as f:
        if f.read() == out:
            sys.exit(0)
except FileNotFoundError:
    pass
with open(dest + ".tmp", "wb") as f:
    f.write(out)
os.replace(dest + ".tmp", dest)
PY
}

# Claims interface 0 through usbfs as another program would, until
# logicpro_release (as logic16_hold in saleae-logic16.sh).
logicpro_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device, nothing to claim
    timeout 120 python3 -I -c '
import fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/logicpro-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/logicpro-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "saleae-logic-pro: could not claim the interface: $(cat "$TMP/logicpro-hold")"
            return 1
        fi
        sleep 0.1
    done
}

logicpro_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

# The model's options for the cases that follow (see fx2lafw.sh).
logicpro_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "saleae-logic-pro: the emulator refused '$*'"
}

emu_cases() {
    F=saleae-logic-pro
    logicpro_fx3_standin || emu_fail "saleae-logic-pro: could not write the FX3 stand-in"
    # The driver's log lines under both tools' prefixes. What goes, all
    # wording of the frame around the driver's lines: oxrok's module path
    # for libsigrok's driver name; libsigrok's "Opening device instance",
    # its session's, hwdriver's and resource.c's lines, and oxrok's
    # one line for a missing file, with its hint, against resource.c's two;
    # oxrok's refusal of the limit under hwdriver's name, which then goes
    # the same way; the cause oxrok adds to "Failed to open device." and
    # "Failed to start session." (the driver logged the cause itself, which
    # stays). The byte of two messages is masked as the C prints the wrong
    # one, only for the pair each case can show (protocol.c:341 the count
    # 0x04 for the status 0x00, protocol.c:540 rsp[0] 0x00 for reg_val 0x55),
    # so another byte fails; oxrok's is pinned by unit tests.
    N='s/ \[[0-9:.]+\]//; s/^oxrok: err: oxrok::cli::run: /sr: hwdriver: /; s/^oxrok: [a-z]+: oxrok::drivers::saleae_logic_pro[a-z0-9_:]*: /sr: saleae-logic-pro: /; /^sr: saleae-logic-pro: firmware file not found/d; /^sr: device: saleae-logic-pro: Opening device instance/d; /^sr: (session|hwdriver|resource): /d; s/^Failed to open device.*/Failed to open device./; s/^(Unknown sample rate|device disconnected).*/Failed to start session./; s/(I2C wake read \()0x(04|00)/\1BYTE/; s/(read-back \()0x(00|55)/\1BYTE/'
    # A renumerated device's address, in the text the harness's conn=
    # rule does not reach (--show, --get).
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'
    # Each transfer of 16 KiB holds 131072 samples of one channel, 8192
    # of sixteen: the goldens keep the first line of each channel and the
    # digest of the whole output.
    B1='1,/^0:/!d'
    B4='1,/^3:/!d'
    B16='1,/^15:/!d'
    D4=0,1,2,3
    # The address of the session's first attach, the board without its
    # firmware: the uploads below move it elsewhere (EMU_USB follows).
    A0=$EMU_USB

    # Without firmware: every run uploads it in the scan, sleeps 500 ms
    # and lists the renumerated device (scan(), api.c:213-218): the
    # device must be back within the 500 ms, which the emulator's
    # re-attach is.
    # A by-address case on this board cannot run full: the first tool's
    # upload moves it, so the second tool's address is stale. sigrok-cli
    # lists it all the same through the conn= hole (api.c:219-236); oxrok
    # is held to following it by this case, under --no-oracle only (oxrok
    # alone, on the session's first attach), with goldens copied from the
    # run by ids below: the same scan, by address.
    logicpro_options
    L="$N; /saleae-logic-pro:/!d; s/ \[[0-9:.]+\]//"
    emu_case "by address unflashed" --no-oracle-only --stderr --filter "$L" -- -l 4 -d $F:conn=$A0 --scan
    emu_case "by ids unflashed" --stderr --filter "$L" -- -l 4 -d $F:conn=21a9.1006 --scan
    emu_case "scan" --keep-header --filter "$C" -- -d $F --scan
    # sr_usb_find() refuses the spec, and the driver lists the board all
    # the same (the stdout case below shows it).
    # oxrok lists nothing for a conn= that names nothing (api.c:219-236).
    W="api.c:222 filters only when sr_usb_find() found something: if (conn_devices) ... so a conn that matches nothing, or that it refuses, selects every Logic Pro; api.c:194-212 uploads to every board whatever the conn"
    emu_case "bad conn" --oracle-differs "$W" --stderr --filter 's/^sr: usb: //; s/^oxrok: [a-z]+: [a-z0-9_:]+: //' -- -d $F:conn=nonsense --scan
    emu_case "bad conn lists" --oracle-differs "$W" --keep-header --no-output --filter "$C" -- -d $F:conn=nonsense --scan
    emu_case "show" --keep-header --filter "$C" -- -d $F --show
    emu_case "by ids" --digest --filter "$B4" -- -d $F:conn=21a9.1006 -C $D4 --continuous -O bits
    emu_case "unflashed 16ch 10M" --digest --filter "$B16" -- -d $F --continuous -O bits
    # The boot stage's strings are not known: with strings, each
    # strcmp() of scan_firmware() fails in turn, and the upload follows.
    logicpro_options boot_strings=Cypress/WestBridge
    emu_case "boot strings other vendor" --keep-header --filter "$C" -- -d $F --scan
    logicpro_options boot_strings=Saleae/Bootloader
    emu_case "boot strings other product" --keep-header --filter "$C" -- -d $F --scan

    # By its real address on a board just attached again: back to the
    # boot stage, then the firmware running, whose descriptors differ, so
    # without emu_reset the case's own reset would move the board after
    # the case line read EMU_USB. libsigrok's debug line shows
    # sr_usb_find() matching it. (Without firmware no conn= can match:
    # the upload moves the board before sr_usb_find() runs.)
    logicpro_options
    emu_reset
    logicpro_options flashed=1
    emu_reset
    emu_case "by address after a reset" --keep-header --filter "$C" -- -d $F:conn=$EMU_USB --scan
    # The firmware running: no upload, and the scan reads the strings.
    emu_case "flashed scan" --keep-header --filter "$C" -- -d $F --scan
    # An address no device has, and a bus and an address out of range
    # (sr_usb_split_conn() refuses them): listed all the same.
    emu_case "conn matching nothing" --oracle-differs "$W" --keep-header --no-output --filter "$C" -- -d $F:conn=9.99 --scan
    emu_case "conn bus out of range" --oracle-differs "$W" --keep-header --no-output --filter "$C" -- -d $F:conn=300.1 --scan
    emu_case "conn address out of range" --oracle-differs "$W" --keep-header --no-output --filter "$C" -- -d $F:conn=1.200 --scan
    # A conn= that names no device any more: A0 is the address of the
    # session's first attach, which the uploads above moved the board
    # away from, so sr_usb_find() matches nothing and the board is
    # captured only because such a conn selects every Logic Pro.
    emu_case "stale address captures all the same" --oracle-differs "$W" --expect-fail --stderr --digest --filter "$B4" -- -d $F:conn=$A0 -C $D4 --continuous -O bits
    # Selected by its real address (EMU_USB, the board as it is now):
    # the previous case left the board with these descriptors, so the
    # next case's reset does not re-attach it and the address still
    # holds. libsigrok's debug line shows sr_usb_find() matching it.
    A=$EMU_USB
    emu_case "by address" --keep-header --filter "$C" -- -d $F:conn=$A --scan
    emu_case "by address capture" --digest --filter "$B4" -- -d $F:conn=$A -C $D4 --continuous -O bits
    emu_case "get conn" --filter "s/'[0-9]+[.][0-9]+'/'BUS.ADDR'/" -- -d $F --get conn
    emu_case "get samplerate" -- -d $F --get samplerate
    emu_case "1ch 1M" --digest --filter "$B1" -- -d $F -C 0 -c samplerate=1M --continuous -O bits
    emu_case "3ch 2M" --digest --filter '1,/^15:/!d' -- -d $F -C 0,7,15 -c samplerate=2M --continuous -O bits
    emu_case "4ch 2.5M" --digest --filter "$B4" -- -d $F -C $D4 -c samplerate=2500k --continuous -O bits
    emu_case "8ch 10M" --digest --filter '1,/^15:/!d' -- -d $F -C 8,9,10,11,12,13,14,15 --continuous -O bits
    emu_case "16ch 25M" --digest --filter "$B16" -- -d $F -c samplerate=25M --continuous -O bits
    emu_case "5ch 50M" --digest --filter '1,/^14:/!d' -- -d $F -C 1,3,5,9,14 -c samplerate=50M --continuous -O bits
    emu_case "16ch 50M hex" --digest --filter '1,/^15:/!d' -- -d $F -c samplerate=50M --continuous -O hex
    # The csv header holds the time: none.
    emu_case "csv" --digest --filter '1,/^0,/!d' -- -d $F -C 0,1 -c samplerate=1M --continuous -O csv:header=false
    # The driver lists no sample or time limit (api.c devopts).
    emu_case "samples refused" --expect-fail --stderr -- -d $F -C 0 --samples 100 -O bits
    emu_case "time refused" --expect-fail --stderr --filter "$N" -- -d $F -C 0 --time 10 -O bits
    # config_set() takes any rate; saleae_logic_pro_prepare() refuses it.
    # The session then stops the device it could not start: the stop
    # sequence goes out all the same.
    emu_case "bad rate" --expect-fail --stderr --filter "$N" -- -d $F -C 0 -c samplerate=3M --continuous -O bits
    logicpro_options flashed=1 transfers=5
    emu_case "five transfers" --digest --filter "$B1" -- -d $F -C 4 -c samplerate=50M --continuous -O bits
    logicpro_options flashed=1 fpga=1
    emu_case "fpga configured" --digest --filter "$B4" -- -d $F -C $D4 --continuous -O bits
    # The recorded captures: a transfer from a board with its FPGA up, and
    # one that takes the bitstream first (tests/fixtures/saleae_logic_pro).
    logicpro_options flashed=1 fpga=1 transfers=1
    emu_case "recorded capture" --record --digest --filter '1,/^3:/!d' -- -d $F -C $D4 --continuous -O hex
    logicpro_options flashed=1 transfers=1
    emu_case "recorded bitstream upload" --record --digest --filter '1,/^0:/!d' -- -d $F -C 0 --continuous -O hex
    logicpro_options flashed=1 fpga=1
    # The driver's debug lines: the firmware version, the temperature
    # and the authentication (the values are the model's invention).
    emu_case "debug lines" --stderr --filter "$N"'; /saleae-logic-pro: (fw-version|temperature|random|nonce|sig|Authenticate|[0-9]+ channels|Skipping)/!d; s/ \[[0-9:.]+\]//' -- -l 5 -d $F -C 0,1 --continuous -O bits

    # A claim held by another program: dev_open() fails to claim.
    # emu_case begins with a reset, which re-attaches the board when the
    # previous case left it with other descriptors (boot stage against
    # firmware); the claim would then be on the device that left. So the
    # hold needs the previous case to end with the same descriptors as
    # this one: here the firmware's, as "debug lines" leaves them.
    logicpro_options flashed=1 fpga=1
    if logicpro_hold; then
        emu_case "interface claimed by another program" --expect-fail --stderr --filter "$N; /saleae-logic-pro:|Failed to open/!d; s/ \[[0-9:.]+\]//" -- -l 4 -d $F -C 0 --continuous -O bits
        logicpro_release
    fi

    # Faults. The board starts with its firmware, and with its FPGA
    # configured unless the fault is in the upload.
    # Init fails: the open fails with the driver's debug line.
    # -l 5 for the debug line of the failing request; the LFSR steps
    # (0 to 0 before the authentication) and the chunk sizes go.
    E="$N"'; /saleae-logic-pro:|Failed to open/!d; /Iterate|Uploading [0-9]+ bytes/d; s/ \[[0-9:.]+\]//'
    for f in reseed fw-version read-eeprom read-reg init-bitstream send-bitstream; do
        logicpro_options flashed=1 stall=$f
        emu_case "fault stall $f" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    done
    for f in init-bitstream send-bitstream; do
        logicpro_options flashed=1 bad_status=$f
        emu_case "fault status $f" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    done
    # After the upload: the scratch write (the first register write) and
    # its read back (the second register read).
    logicpro_options flashed=1 stall=write-reg:1
    emu_case "fault stall scratch write" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 stall=read-reg:2
    emu_case "fault stall scratch read" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    # An FPGA that did not come up: the message prints the init answer
    # (rsp[0], 0x00), not the register (protocol.c:540).
    logicpro_options flashed=1 scratch=0x55
    emu_case "fault fpga not up" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    # Every write of saleae_logic_pro_init() with the FPGA configured:
    # the ADC reset (1, 2), the ADC set up (3-12), register 0x15 (13),
    # ADC 0x0f (14), the resets (15-20), the first magic byte (21).
    for n in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21; do
        logicpro_options flashed=1 fpga=1 stall=write-reg:$n
        emu_case "fault stall init write $n" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    done
    logicpro_options flashed=1 fpga=1 stall=read-reg:2
    emu_case "fault stall dummy read" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 fpga=1 stall=read-eeprom:2
    emu_case "fault stall magic read" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 fpga=1 stall=read-temp
    emu_case "fault stall temperature" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 fpga=1 short_reply=fw-version
    emu_case "fault short fw version" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 fpga=1 stall_reply=read-eeprom
    emu_case "fault stall eeprom answer" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits

    # The authentication fails: saleae_logic_pro_prepare() ignores it
    # (protocol.c:783), both sides keep the plain state, and the
    # capture runs. Only the debug line of the failure differs.
    G="$N"'; /saleae-logic-pro: (Failed|Authenticate)/!d; s/ \[[0-9:.]+\]//'
    for f in wake-i2c read-i2c:1 write-i2c:1 read-i2c:2 write-i2c:2 read-i2c:3 write-i2c:3 read-i2c:4 write-i2c:4 read-i2c:5; do
        logicpro_options flashed=1 fpga=1 stall_reply=$f
        emu_case "fault auth stall reply $f" --stderr --filter "$G" -- -l 5 -d $F -C 0 --continuous -O bits
    done
    for f in wake-i2c write-i2c read-i2c; do
        logicpro_options flashed=1 fpga=1 bad_status=$f
        emu_case "fault auth status $f" --stderr --filter "$G" -- -l 5 -d $F -C 0 --continuous -O bits
    done
    # The chip's wake answer: the message prints the count byte, not
    # the status (protocol.c:341).
    logicpro_options flashed=1 fpga=1 wake=0x00
    emu_case "fault auth wake answer" --stderr --filter "$G" -- -l 5 -d $F -C 0 --continuous -O bits

    # The stop: the status read fails, or reads a capture error; the
    # driver logs it and the capture ends as usual.
    logicpro_options flashed=1 fpga=1 stall_reply=read-reg:10
    emu_case "fault stop status read" --digest --filter "$B1" -- -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 fpga=1 capture_status=0x00
    emu_case "fault capture error" --stderr --filter "$N; /Capture error/!d" -- -d $F -C 0 --continuous -O bits

    # Ctrl-C: sigrok-cli has no SIGINT handler, while oxrok sends the stop's
    # requests and ends the capture. The idle timeout would end it as well
    # (the model stops after its transfers), so this shows the path through
    # the CLI, and a unit test (a_stop_ends_a_stream_that_goes_on) that the
    # flag ends a stream that does not stop. How much output comes
    # before the stop depends on time: only the first line counts.
    logicpro_options flashed=1 fpga=1 transfers=3
    emu_case "interrupt during a capture" --interrupt in=2:3 --oracle-bug "sigrok-cli has no SIGINT handler" --filter '2,$d' -- -d $F -C 0 --continuous -O bits

    # The driver's own idle stop (api.c:391): the runner has no any-key
    # stop, so nothing else ends the capture after the model's transfers.
    # sigrok-cli's stdin is /dev/null, so it never gets here.
    logicpro_options flashed=1 fpga=1
    emu_case "lib idle stop" --libsigrok --oxrok --digest --filter "$B1" -- -d $F -C 0 --continuous -O bits

    # oxrok's --strict makes the warning of a failed authentication a
    # failed run; sigrok-cli has no such flag.
    logicpro_options flashed=1 fpga=1 bad_status=wake-i2c
    emu_case "strict on a failed authentication" --no-oracle-only --expect-fail --stderr --filter "$N; /No authentication|treated as error/!d" -- --strict -d $F -C 0 --continuous -O bits

    # The sample stream.
    logicpro_options flashed=1 fpga=1 halt_in=1
    # The transfer that brought the data goes back in from its callback, in
    # the C driver's event handling inside the blocking write of the start
    # (protocol.c:909), before or after that write's answer is asked for:
    # its STALL lands on either side of the answer, by timing. The stalled
    # IN lines between the data and the next write move to the end of the
    # log, so their number still counts.
    emu_case "fault halt after one" --tx-filter '/^bulk_in 82 16384 -> @/,/^bulk_out/{/err stall/{H;d}}; ${G}' --digest --filter "$B1" -- -d $F -C 0 --continuous -O bits
    logicpro_options flashed=1 fpga=1 halt_in=0
    emu_case "fault halt at once" --no-output -- -d $F -C 0 --continuous -O bits
    # A short transfer: receive_data() converts all 16 KiB of the buffer
    # whatever the transfer's length (protocol.c:906), so 4 bytes of
    # samples come out as 131072 samples, all but the first 32 from a
    # buffer never filled, whose content is undefined. The filter keeps
    # the header, the first line (32 samples, the 4 bytes sent) and the
    # number of the last line: 4096 lines of 32 samples after the header.
    logicpro_options flashed=1 fpga=1 short_in=4 transfers=1
    emu_case "fault short transfer" --oracle-differs "protocol.c:906 converts 16 * 1024 / 4 words whatever transfer->actual_length" --filter '1,2p; $=; d' -- -d $F -C 0 --continuous -O bits:width=32
    # Unplugged at the third transfer: the transfers then end with
    # LIBUSB_TRANSFER_NO_DEVICE, which the driver drops, and the stop's
    # requests fail; how many of the driver's debug lines that makes
    # depends on timing, so only the first line of the output is kept.
    logicpro_options flashed=1 fpga=1 transfers=4
    emu_case "unplug mid capture" --unplug in=2:3 --filter '2,$d' -- -d $F -C 0,1 --continuous -O bits

    # The scan cannot read the running firmware's strings: it takes it
    # for the boot loader and uploads, which the firmware refuses; the
    # board is listed and runs all the same.
    U="$N"'; /saleae-logic-pro:|Failed/!d; s/ \[[0-9:.]+\]//'
    logicpro_options flashed=1 fpga=1 stall_string=1
    emu_case "fault manufacturer unreadable" --stderr --filter "$U" -- -l 4 -d $F --scan
    logicpro_options flashed=1 fpga=1 short_string=2
    emu_case "fault product unreadable" --stderr --filter "$U" -- -l 4 -d $F --scan
    logicpro_options flashed=1 fpga=1 stall_string=2
    emu_case "fault product unreadable capture" --digest --filter "$B1" -- -d $F -C 0 --continuous -O bits
    # The boot loader refuses a part: the upload fails, and the second
    # pass lists the board in its boot stage, whose open fails.
    logicpro_options stall_load=2
    emu_case "fault boot loader refuses" --expect-fail --stderr --filter "$U" -- -l 4 -d $F -C 0 --continuous -O bits
    # Unplugged during the upload (the Nth control write after the
    # SET_CONFIGURATION). The second pass finds the board or not as the
    # kernel's removal goes, and libusb's own line then says which error
    # it met (1 run in 5): only the driver's and sigrok-cli's lines count.
    logicpro_options
    emu_case "unplug during the upload" --unplug out=0:3 --expect-fail --stderr --filter "$U; /^sr: usb: Failed to open device/d" -- -l 4 -d $F -C 0 --continuous -O bits

    # Unplugged during saleae_logic_pro_prepare(), after the write of
    # register 0x15 (the 61st command): the prepare ignores the failing
    # requests, dev_acquisition_start() then fails to submit its
    # transfers and gives up (api.c:421-427).
    logicpro_options flashed=1 fpga=1
    emu_case "unplug during prepare" --unplug out=1:61 --expect-fail --stderr --filter "$N; /Failed to submit|Failed to start session/!d; s/ \[[0-9:.]+\]//" -- -d $F -C 0 --continuous -O bits

    # The bitstream file missing: upload_bitstream() fails (moved away
    # for the case, then back).
    logicpro_options flashed=1
    if mv "$SIGROK_FIRMWARE_DIR/saleae-logicpro16-fpga.bitstream" "$TMP/logicpro-bitstream"; then
        emu_case "bitstream file missing" --expect-fail --stderr --filter "$E" -- -l 5 -d $F -C 0 --continuous -O bits
        mv "$TMP/logicpro-bitstream" "$SIGROK_FIRMWARE_DIR/saleae-logicpro16-fpga.bitstream" ||
            emu_fail "saleae-logic-pro: could not put the bitstream stand-in back"
    fi

    # Broken firmware files: upload_firmware() refuses them before or
    # after sending parts, and the board stays in its boot stage, which
    # the scan's second pass lists all the same; its open then fails.
    logicpro_options
    for how in big truncated nofinal missing; do
        logicpro_fx3_standin $how || emu_fail "saleae-logic-pro: could not write the $how FX3 file"
        emu_case "firmware file $how" --expect-fail --stderr --filter "$U" -- -l 4 -d $F -C 0 --continuous -O bits
    done
    # The last part empty but not flagged final: the boot loader starts
    # the firmware, and both tools list the running board. libsigrok calls
    # that a success with its "Missing final part." (api.c:111-116, ret is
    # the transfer's 0), oxrok a failed upload: stderr differs, not this.
    logicpro_fx3_standin unflagged || emu_fail "saleae-logic-pro: could not write the unflagged FX3 file"
    emu_case "firmware file unflagged" --keep-header --filter "$C" -- -d $F --scan
    logicpro_fx3_standin || emu_fail "saleae-logic-pro: could not write the FX3 stand-in back"
}
