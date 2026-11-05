# The UNI-T UT-D04 cable on raw USB, libsigrok's uni-t-dmm driver
# (emu/src/models/uni-t-dmm.rs). Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model".
#
# One scenario per chipset of api.c's table (es519xx 11 and 14 byte,
# fs9721, fs9922, ut372, ut71x, vc870) and one sample run per driver id,
# so the table's rates, parsers, dmm_details and the parity mask are each
# compared. The packets are written from the layouts in src/dmm/*.c.
# Every packet goes through the driver's own framing: a chunk is 8 bytes
# on endpoint 0x82, one IN per poll, so counts are the driver's.
EMU_MODEL=uni-t-dmm
EMU_OPTIONS=""
EMU_DRIVER=uni-t-ut61e
EMU_GCOV=uni-t-dmm
# oxrok has the driver: its cases must run full.
EMU_FULL="tenma-72-7750 uni-t-ut60g uni-t-ut61e tecpel-dmm-8061 tenma-72-7745 uni-t-ut60a uni-t-ut60e voltcraft-vc820 voltcraft-vc840 uni-t-ut61b uni-t-ut61c uni-t-ut61d voltcraft-vc830 uni-t-ut372 tenma-72-7730 tenma-72-7732 tenma-72-9380a uni-t-ut71a uni-t-ut71b uni-t-ut71c uni-t-ut71d uni-t-ut71e uni-t-ut804 voltcraft-vc920 voltcraft-vc940 voltcraft-vc960 voltcraft-vc870"
EMU_GCOV_SHARED="src/usb.c src/dmm/es519xx.c src/dmm/fs9721.c src/dmm/fs9922.c src/dmm/ut372.c src/dmm/ut71x.c src/dmm/vc870.c"

# The model's options for the cases that follow; none restores
# EMU_OPTIONS (see tools/emu/ftdi-la.sh).
unit_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "uni-t-dmm: the emulator refused '$*'"
}

# Takes usbhid off interface 0, as a capture before it would have, for a
# case that must not depend on the order of the cases: USBDEVFS_IOCTL with
# USBDEVFS_DISCONNECT, which sends nothing to the device.
unit_disconnect() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: no device
    python3 -I -c '
import array, fcntl, os, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
buf = array.array("B", struct.pack("iiQ", 0, 0x5516, 0))
try:
    fcntl.ioctl(fd, 0xc0105512, buf)
except OSError:
    pass  # no driver bound
' "$EMU_BUSID" || emu_fail "uni-t-dmm: could not disconnect usbhid"
}

# Claims interface 0 through usbfs as another program would, until
# unit_release. (As hantek_hold in tools/emu/hantek-6xxx.sh.)
unit_hold() {
    [ "$EMU_DRY" = 0 ] || return 0 # diffcli counting cases: nothing to claim
    timeout 120 python3 -I -c '
import array, fcntl, os, signal, struct, sys
d = "/sys/bus/usb/devices/" + sys.argv[1]
bus, dev = (int(open(d + f).read()) for f in ("/busnum", "/devnum"))
fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_RDWR)
# usbhid, when no capture has detached it yet (a -k run): USBDEVFS_IOCTL
# with USBDEVFS_DISCONNECT, which sends nothing to the device.
buf = array.array("B", struct.pack("iiQ", 0, 0x5516, 0))
try:
    fcntl.ioctl(fd, 0xc0105512, buf)
except OSError:
    pass  # no driver bound
fcntl.ioctl(fd, 0x8004550f, struct.pack("I", 0))  # USBDEVFS_CLAIMINTERFACE
print("held", flush=True)
signal.pause()
' "$EMU_BUSID" >"$TMP/unit-hold" 2>&1 &
    HOLDPID=$!
    i=0
    until grep -q '^held$' "$TMP/unit-hold"; do
        i=$((i + 1))
        if [ $i -gt 100 ] || ! kill -0 $HOLDPID 2>/dev/null; then
            emu_fail "uni-t-dmm: could not claim the interface: $(cat "$TMP/unit-hold")"
            return 1
        fi
        sleep 0.1
    done
}

unit_release() {
    kill $HOLDPID 2>/dev/null
    wait $HOLDPID 2>/dev/null
    HOLDPID=""
}

emu_cases() {
    # es519xx, 14 bytes, 19200/7o1: 12.345 V, 20.456 V. The model adds the
    # parity bit, as the chip passes it.
    ES14=3131323334353b3030303a300d0a,3132303435363b3030303a300d0a
    # es519xx, 11 bytes twice over, 19200/7o1.
    ES11A=3131323334 ES11B=3b3030300d0a
    ES11=$ES11A$ES11B$ES11A$ES11B
    # fs9721, 2400/8n1: 1.234 V DC; the same digits with bit c2c1_00 and
    # no unit, which the "00_temp_c" meters read as a temperature; -12.34
    # kOhm.
    FS21=1720354d5b617f8297a0b0c0d4e0,1720354d5b617f8297a0b0c0d0e1,172835455b697f8297a2b0c4d0e0
    # fs9922, 2400/8n1: +1.234 V DC; the z1 "diode" of the VC-830 with no
    # unit bits; an overload in ohm.
    FS22=2b31323334203130000080000d0a,2b30353637203130800000000d0a,2b3f303a3f203130000020000d0a
    # ut372, 2400/8n1: 12345 RPM, and 1234.5 as a count with its point.
    UT372=30333d3635373c353e363030303030303030303030303030310d0a,30333d3e35373c353e363030303030303030303030303030320d0a
    # ut71x, 2400/7o1: 1.2345 V DC, 10.000 kOhm.
    UT71=3132333435313132300d0a,3130303030333430300d0a
    # vc870, 9600/8n1: 12.345 V DC, 1.0000 MOhm.
    VC870=3030313132333435303030303030303030303030300d0a,3230333130303030303030303030303030303030300d0a

    es14() { unit_options "packet=$ES14" baud=19200 parity=odd; }
    es11() { unit_options "packet=$ES11" baud=19200 parity=odd; }
    fs21() { unit_options "packet=$FS21" baud=2400 parity=none; }
    fs22() { unit_options "packet=$FS22" baud=2400 parity=none; }
    ut372() { unit_options "packet=$UT372" baud=2400 parity=none; }
    ut71() { unit_options "packet=$UT71" baud=2400 parity=odd; }
    vc870() { unit_options "packet=$VC870" baud=9600 parity=none; }

    # oxrok's log prefix as libsigrok's; the message itself is compared.
    N='s/^oxrok: [a-z]+: oxrok::drivers::uni_t_dmm: (Invalid input|Could neither)/sr: usb: \1/
s/^oxrok: [a-z]+: oxrok::drivers::uni_t_dmm: /sr: uni-t-dmm: /'
    U=uni-t-ut61e:conn=$EMU_USB
    I=uni-t-ut61e:conn=1a86.e008
    # Selection.
    es14
    emu_case "scan by address" --keep-header -- -d $U --scan
    emu_case "scan by ids" --keep-header -- -d $I --scan
    emu_case "scan other ids" --keep-header --no-output -- -d uni-t-ut61e:conn=1a86.e009 --scan
    emu_case "scan bad conn" --stderr --filter "$N
/For a serial port, use/d" -- -d uni-t-ut61e:conn=notusb --scan
    # The hint oxrok adds there, which the case above filters.
    emu_case "scan bad conn hint" --oracle-differs "oxrok names the driver for a serial port, sigrok-cli does not" --stderr --filter "$N" -- -d uni-t-ut61e:conn=notusb --scan
    # An option the C does not list: hwdriver.c check_options refuses it,
    # whatever the driver. Each tool's own line goes; the result stays.
    emu_case "serialcomm on raw usb" --expect-fail --stderr --filter "$N
/Invalid option .serialcomm./d
/serialcomm does not apply/d" -- -d uni-t-ut61e:conn=1a86.e008:serialcomm=2400/8n1 --samples 1
    emu_case "show" --keep-header --filter 's/^(    conn: )[0-9]+\.[0-9]+/\1BUS.ADDR/' -- -d $I --show
    # oxrok's message says why after "invalid argument"; cut there.
    emu_case "get limit_samples" --expect-fail --stderr --filter 's/(invalid argument).*/\1/' -- -d $I --get limit_samples
    emu_case "set limit_samples" --no-output -- -d $I --set -c limit_samples=5

    # The captures of each chipset.
    es14
    emu_case "es519xx14 samples 1" -- -d $I --samples 1
    emu_case "es519xx14 samples 4" --record -- -d $I --samples 4
    emu_case "es519xx14 csv" -- -d $I --samples 2 -O csv:header=false
    emu_case "es519xx14 by address" -- -d $U --samples 2
    # No conn=: nothing in the C (api.c:73). oxrok looks for the cable
    # through the HID layer, which a raw capture takes away from it, so
    # it finds nothing either. A cable with usbhid on it is
    # tools/emu/ch9325.sh's.
    unit_disconnect
    emu_case "scan no conn" --keep-header --no-output -- -d uni-t-ut61e --scan
    es11
    emu_case "es519xx11 samples 2" -- -d uni-t-ut60g:conn=1a86.e008 --samples 2
    fs21
    emu_case "fs9721 samples 3" --record -- -d uni-t-ut60e:conn=1a86.e008 --samples 3
    fs22
    emu_case "fs9922 samples 3" -- -d uni-t-ut61b:conn=1a86.e008 --samples 3
    emu_case "fs9922 z1 diode" -- -d voltcraft-vc830:conn=1a86.e008 --samples 3
    ut372
    emu_case "ut372 samples 2" -- -d uni-t-ut372:conn=1a86.e008 --samples 2
    ut71
    emu_case "ut71x samples 2" -- -d uni-t-ut71e:conn=1a86.e008 --samples 2
    vc870
    emu_case "vc870 samples 2" -- -d voltcraft-vc870:conn=1a86.e008 --samples 2

    # Every id of api.c's table, one sample each: its rate, parser and
    # details.
    es11
    for id in tenma-72-7750 uni-t-ut60g; do
        emu_case "id $id" -- -d $id:conn=1a86.e008 --samples 2
    done
    es14
    emu_case "id uni-t-ut61e" -- -d uni-t-ut61e:conn=1a86.e008 --samples 2
    fs21
    for id in tecpel-dmm-8061 tenma-72-7745 uni-t-ut60a uni-t-ut60e voltcraft-vc820 voltcraft-vc840; do
        emu_case "id $id" -- -d $id:conn=1a86.e008 --samples 2
    done
    fs22
    for id in uni-t-ut61b uni-t-ut61c uni-t-ut61d voltcraft-vc830; do
        emu_case "id $id" -- -d $id:conn=1a86.e008 --samples 2
    done
    ut71
    for id in tenma-72-7730 tenma-72-7732 tenma-72-9380a uni-t-ut71a uni-t-ut71b uni-t-ut71c uni-t-ut71d uni-t-ut71e uni-t-ut804 voltcraft-vc920 voltcraft-vc940 voltcraft-vc960; do
        emu_case "id $id" -- -d $id:conn=1a86.e008 --samples 2
    done

    # A sweep per chipset: the functions and the flag bits of its layout,
    # one at a time, so the parsers' tables are compared too. Generated
    # from src/dmm/*.c's layouts; what a packet means is the goldens'.
    unit_options packet=3131323334353d3030303a300d0a,3131323334353f3030303a300d0a,313132333435303030303a300d0a,313132333435393030303a300d0a,313132333435333030303a300d0a,313132333435353030303a300d0a,313132333435313030303a300d0a,313132333435323030303a300d0a,313132333435363030303a300d0a,3131323334353e3030303a300d0a,3131323334353b3130303a300d0a,3131323334353b3230303a300d0a,3131323334353b3430303a300d0a,3131323334353b3830303a300d0a,3131323334353b3031303a300d0a,3131323334353b3032303a300d0a,3131323334353b3034303a300d0a,3131323334353b3038303a300d0a,3131323334353b3030313a300d0a,3131323334353b3030323a300d0a,3131323334353b3030343a300d0a,3131323334353b3030383a300d0a,3131323334353b30303031300d0a,3131323334353b30303032300d0a,3131323334353b30303034300d0a,3131323334353b30303038300d0a,3131323334353b3030303a310d0a,3131323334353b3030303a320d0a,3131323334353b3030303a340d0a,3131323334353b3030303a380d0a,3031323334353b3030303a300d0a,3231323334353b3030303a300d0a,3331323334353b3030303a300d0a,3431323334353b3030303a300d0a baud=19200 parity=odd idle=0
    emu_case "sweep es14" -- -d uni-t-ut61e:conn=1a86.e008 --samples 34
    unit_options packet=31313233343d3030300d0a31313233343d3030300d0a,31313233343f3030300d0a31313233343f3030300d0a,3131323334303030300d0a3131323334303030300d0a,3131323334393030300d0a3131323334393030300d0a,3131323334333030300d0a3131323334333030300d0a,3131323334353030300d0a3131323334353030300d0a,3131323334313030300d0a3131323334313030300d0a,3131323334323030300d0a3131323334323030300d0a,3131323334363030300d0a3131323334363030300d0a,31313233343e3030300d0a31313233343e3030300d0a,31313233343b3130300d0a31313233343b3130300d0a,31313233343b3230300d0a31313233343b3230300d0a,31313233343b3430300d0a31313233343b3430300d0a,31313233343b3830300d0a31313233343b3830300d0a,31313233343b3031300d0a31313233343b3031300d0a,31313233343b3032300d0a31313233343b3032300d0a,31313233343b3034300d0a31313233343b3034300d0a,31313233343b3038300d0a31313233343b3038300d0a,31313233343b3030310d0a31313233343b3030310d0a,31313233343b3030320d0a31313233343b3030320d0a,31313233343b3030340d0a31313233343b3030340d0a,31313233343b3030380d0a31313233343b3030380d0a baud=19200 parity=odd idle=0
    emu_case "sweep es11" -- -d uni-t-ut60g:conn=1a86.e008 --samples 22
    unit_options packet=1120354d5b617f8297a0b0c0d4e0,1220354d5b617f8297a0b0c0d4e0,1420354d5b617f8297a0b0c0d4e0,1820354d5b617f8297a0b0c0d4e0,1720354d5b617f8297a1b0c0d4e0,1720354d5b617f8297a2b0c0d4e0,1720354d5b617f8297a4b0c0d4e0,1720354d5b617f8297a8b0c0d4e0,1720354d5b617f8297a0b1c0d4e0,1720354d5b617f8297a0b2c0d4e0,1720354d5b617f8297a0b4c0d4e0,1720354d5b617f8297a0b8c0d4e0,1720354d5b617f8297a0b0c1d4e0,1720354d5b617f8297a0b0c2d4e0,1720354d5b617f8297a0b0c4d4e0,1720354d5b617f8297a0b0c8d4e0,1720354d5b617f8297a0b0c0d1e0,1720354d5b617f8297a0b0c0d2e0,1720354d5b617f8297a0b0c0d4e0,1720354d5b617f8297a0b0c0d8e0,1720354d5b617f8297a0b0c0d4e1,1720354d5b617f8297a0b0c0d4e2,1720354d5b617f8297a0b0c0d4e4,1720354d5b617f8297a0b0c0d4e8 baud=2400 parity=none idle=0
    emu_case "sweep fs21" -- -d uni-t-ut60e:conn=1a86.e008 --samples 24
    unit_options packet=2b31323334203030000080000d0a,2b31323334203230000080000d0a,2b31323334203430000080000d0a,2b31323334203101000080000d0a,2b31323334203102000080000d0a,2b31323334203104000080000d0a,2b31323334203108000080000d0a,2b31323334203110000080000d0a,2b31323334203120000080000d0a,2b31323334203140000080000d0a,2b31323334203180000080000d0a,2b31323334203130010080000d0a,2b31323334203130020080000d0a,2b31323334203130040080000d0a,2b31323334203130080080000d0a,2b31323334203130100080000d0a,2b31323334203130200080000d0a,2b31323334203130400080000d0a,2b31323334203130800080000d0a,2b31323334203130000180000d0a,2b31323334203130000280000d0a,2b31323334203130000400000d0a,2b31323334203130000880000d0a,2b31323334203130001080000d0a,2b31323334203130002080000d0a,2b31323334203130004080000d0a,2b31323334203130008080000d0a,2b31323334203130000001000d0a,2b31323334203130000002000d0a,2b31323334203130000004000d0a,2b31323334203130000008000d0a,2b31323334203130000010000d0a,2b31323334203130000020000d0a,2b31323334203130000040000d0a,2b31323334203130000080000d0a baud=2400 parity=none idle=0
    emu_case "sweep fs22" -- -d voltcraft-vc830:conn=1a86.e008 --samples 35
    unit_options packet=3132333435313032300d0a,3132333435313132300d0a,3132333435313232300d0a,3132333435313332300d0a,3132333435313432300d0a,3132333435313532300d0a,3132333435313632300d0a,3132333435313732300d0a,3132333435313832300d0a,3132333435313932300d0a,3132333435313a32300d0a,3132333435313b32300d0a,3132333435313c32300d0a,3132333435313d32300d0a,3132333435313e32300d0a,3132333435313f32300d0a,3132333435313130300d0a,3132333435313131300d0a,3132333435313132300d0a,3132333435313133300d0a,3132333435313134300d0a,3132333435313138300d0a,3132333435313132310d0a,3132333435313132320d0a,3132333435313132340d0a,3132333435313132380d0a baud=2400 parity=odd idle=0
    emu_case "sweep ut71" -- -d uni-t-ut71e:conn=1a86.e008 --samples 26
    unit_options packet=3031313132333435303030303030303030303030300d0a,3130313132333435303030303030303030303030300d0a,3131313132333435303030303030303030303030300d0a,3230313132333435303030303030303030303030300d0a,3231313132333435303030303030303030303030300d0a,3330313132333435303030303030303030303030300d0a,3430313132333435303030303030303030303030300d0a,3530313132333435303030303030303030303030300d0a,3531313132333435303030303030303030303030300d0a,3630313132333435303030303030303030303030300d0a,3631313132333435303030303030303030303030300d0a,3730313132333435303030303030303030303030300d0a,3731313132333435303030303030303030303030300d0a,3830313132333435303030303030303030303030300d0a,3831313132333435303030303030303030303030300d0a,3832313132333435303030303030303030303030300d0a,3833313132333435303030303030303030303030300d0a,3930313132333435303030303030303030303030300d0a,3931313132333435303030303030303030303030300d0a,3932313132333435303030303030303030303030300d0a,3933313132333435303030303030303030303030300d0a,3a30313132333435303030303030303030303030300d0a,3a31313132333435303030303030303030303030300d0a,3a32313132333435303030303030303030303030300d0a,3030313132333435303030303030303130303030300d0a,3030313132333435303030303030303230303030300d0a,3030313132333435303030303030303430303030300d0a,3030313132333435303030303030303830303030300d0a,3030313132333435303030303030303031303030300d0a,3030313132333435303030303030303032303030300d0a,3030313132333435303030303030303034303030300d0a,3030313132333435303030303030303038303030300d0a,3030313132333435303030303030303030313030300d0a,3030313132333435303030303030303030323030300d0a,3030313132333435303030303030303030343030300d0a,3030313132333435303030303030303030383030300d0a,3030313132333435303030303030303030303130300d0a,3030313132333435303030303030303030303230300d0a,3030313132333435303030303030303030303430300d0a,3030313132333435303030303030303030303830300d0a,3030313132333435303030303030303030303031300d0a,3030313132333435303030303030303030303032300d0a,3030313132333435303030303030303030303034300d0a,3030313132333435303030303030303030303038300d0a baud=9600 parity=none idle=0
    emu_case "sweep vc870" -- -d voltcraft-vc870:conn=1a86.e008 --samples 44
    unit_options packet=30333d3635373c353e363030303030303030303030303430310d0a,30333d3635373c353e363030303030303030303030303031310d0a,30333d3635373c353e363030303030303030303030303032310d0a,30333d3635373c353e363030303030303030303030303034310d0a,30333d3635373c353e363030303030303030303030303030320d0a,30333d3635373c353e363030303030303030303030303431320d0a baud=2400 parity=none idle=0
    emu_case "sweep ut372" -- -d uni-t-ut372:conn=1a86.e008 --samples 6

    # Overloads, which leave the parsers early: "0.L" with a minus and a
    # point byte that is none; "OL", and under-limit and 4000 count modes.
    unit_options packet=2b3f303a3f203130000020000d0a,2d3f303a3f203130000020000d0a,2b3f303a3f203930000020000d0a,2d3f303a3f203130000080000d0a baud=2400 parity=none idle=0
    emu_case "overload fs9922" -- -d uni-t-ut61b:conn=1a86.e008 --samples 4
    unit_options packet=172030475d66788090a0b0c4d0e0,172830475d66788090a2b0c4d0e0 baud=2400 parity=none idle=0
    emu_case "overload fs9721" -- -d uni-t-ut60a:conn=1a86.e008 --samples 2
    unit_options packet=3a3a303c3a313430300d0a,3a3c303a3a313430300d0a,313233343a313132300d0a baud=2400 parity=odd idle=0
    emu_case "overload ut71x" -- -d uni-t-ut71e:conn=1a86.e008 --samples 3

    # The chunks: a byte per report, and the C header's 1, 2 pattern.
    unit_options "packet=$ES14" baud=19200 parity=odd chunk=1
    emu_case "chunks of 1" -- -d $I --samples 2
    unit_options "packet=$ES14" baud=19200 parity=odd chunk=1,2,7
    emu_case "chunks of 1 2 7" -- -d $I --samples 2
    # The next packet right behind the last, no idle report in between.
    unit_options "packet=$ES14" baud=19200 parity=odd idle=0
    emu_case "no idle reports" -- -d $I --samples 4
    # Noise between packets: misaligned, and the driver resyncs by byte.
    unit_options "packet=$ES14,3f3f3f3f3f" baud=19200 parity=odd
    emu_case "noise resync" -- -d $I --samples 4
    # A packet the validity check takes but the parser refuses (a bad
    # digit): dropped, no sample.
    unit_options "packet=3131323b34353b3030303a300d0a,$ES14" baud=19200 parity=odd
    emu_case "es519xx14 bad digit" -- -d $I --samples 2
    # Parity masked: the same 2400 baud UT71x packets with and without it
    # give the same samples, as the mask does not care.
    unit_options "packet=$UT71" baud=2400 parity=none
    emu_case "ut71x no parity bit" -- -d uni-t-ut71e:conn=1a86.e008 --samples 2

    # A time limit: the meter sends two packets and goes idle, so the
    # idle reports are all that varies with time. They are filtered, the
    # packets are compared.
    NOIDLE='/^int_in 82 .*-> f0/d'
    unit_options "packet=$ES14" baud=19200 parity=odd burst=2
    emu_case "time limit" --tx-filter "$NOIDLE" -- -d $I --time 300
    # The wrong meter: its rate is not the chip's, nothing gets through.
    unit_options "packet=$ES14" baud=19200 parity=odd
    emu_case "wrong rate" --no-output --tx-filter "$NOIDLE" -- -d uni-t-ut61b:conn=1a86.e008 --time 100
    # The right rate, the wrong chipset: no packet is valid.
    fs21
    emu_case "wrong parser" --no-output --tx-filter '/^int_in 82 /d' -- -d uni-t-ut61b:conn=1a86.e008 --time 100
    # A meter that never sends: idle reports only.
    unit_options "packet=$ES14" baud=19200 parity=odd burst=0
    emu_case "meter silent" --no-output --tx-filter "$NOIDLE" -- -d $I --time 100

    # Faults. Each ends the capture at the error, without a df_end from
    # the driver: the session stops as its last source goes.
    unit_options stall-report=1
    emu_case "fault line setting stalled" --oxrok-fails "endpoint stalled" --stderr --filter "$N" -- -d $I --samples 1
    emu_case "fault line setting stalled output" --oxrok-fails "endpoint stalled" --no-output -- -d $I --samples 1
    unit_options stall-in=2
    emu_case "fault in stalled" --oxrok-fails "endpoint stalled" --stderr --filter "$N" -- -d $I --samples 1
    unit_options short-in=2
    emu_case "fault in short" --oxrok-fails "short report" --stderr --filter "$N" -- -d $I --samples 1
    # After 2 samples the chip stops answering: a read times out after 1 s.
    unit_options idle=0 hold=4
    # oxrok's CLI adds a warning of its own, which the C has not.
    emu_case "fault chip hangs" --oxrok-fails "timeout" --stderr --filter "$N
/Device only sent 2 samples/d" -- -d $I --samples 3
    emu_case "fault chip hangs output" --oxrok-fails "timeout" -- -d $I --samples 3
    # The C ends without a df_end there (protocol.c:282); its csv output
    # loses nothing by it.
    emu_case "fault chip hangs csv" --oxrok-fails "timeout" -- -d $I --samples 3 -O csv:header=false
    # The cable pulled in mid capture.
    unit_options
    emu_case "fault unplug" --oxrok-fails "device disconnected" --unplug in=2:3 --stderr --filter "$N" -- -d $I --samples 3

    # A stop in mid capture. The chip then hangs, so the read in progress
    # is what the stop has to wait for; the golden holds no "USB receive
    # error", as a timeout after a stop is no error.
    unit_options idle=0 hold=3
    emu_case "stop during a read" --interrupt in=2:3 --oracle-bug "sigrok-cli has no SIGINT handler: it dies, and api.c's stop is never asked for" --stderr --filter "$N" -- -d $I --samples 100

    # Another program holds the interface: the claim fails (busy).
    unit_options
    emu_case "before the claim" -- -d $I --samples 1
    if unit_hold; then
        emu_case "interface claimed by another program" --oxrok-fails "device is busy" --stderr --filter "$N" -- -d $I --samples 1
        unit_release
    fi
    # A client's stop, between polls: no USB in the C's stop. sigrok-cli
    # cannot ask for one.
    unit_options "packet=$ES14" baud=19200 parity=odd burst=1
    emu_case "lib stop after a sample" --libsigrok --tx-filter "$NOIDLE" -- -d $I --continuous --stop-after 1
    unit_options
}
