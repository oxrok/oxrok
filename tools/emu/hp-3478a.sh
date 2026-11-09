# An HP 3478A multimeter behind an NI GPIB-USB-HS (emu/src/models/hp-3478a.rs), which
# the kernel's ni_usb_gpib carries to a /dev/gpibN, as on a real bench. Sourced by
# tools/diffcli.sh --emu; see docs/USB.md, "A model behind a kernel subsystem".
#
# libsigrok reaches GPIB through linux-gpib's libgpib, which the oracle is built
# without, so there is no sigrok-cli to compare with: every case is --no-oracle-only,
# its goldens oxrok's, written with EMU_WRITE_MISSING=1 and read against the C source
# and HP's manual (the port's module documentation says what differs).
EMU_MODEL=hp-3478a
EMU_DRIVER=hp-3478a
EMU_FULL=$EMU_DRIVER
EMU_GCOV=hp-3478a
# The adapter's interface is the kernel's: keep it bound for oxrok's runs.
EMU_KEEP_BOUND=ni_usb_gpib

# A kernel without the gpib subsystem's NI driver (before 6.13 it was not in
# mainline at all) has no board to carry the model: skipped by name.
modinfo ni_usb_gpib >/dev/null 2>&1 \
    || EMU_SKIP="kernel $(uname -r) has no ni_usb_gpib, so no GPIB board to carry the model"

# What linux-gpib's gpib_config does at a plug, once per attach: a free board's
# type, online, REN, system control and IFC, in a log section of its own. The
# board is the one oxrok-emu took before, or the first with no type, never a
# machine's own (emu/src/gpib_config.rs); the cases name it as EMU_GPIB_MINOR. A
# board that does not come up fails the cases that follow.
emu_attached() {
    emu_cmd "section gpib-config $EMU_BUSID" >/dev/null || return 1
    sudo -n udevadm settle -t 5 || echo "note: hp-3478a: udev has not settled" >&2
    _hpm=$(sudo -n "$EMU" gpib-config --busid "$EMU_BUSID") || return 1
    EMU_GPIB_MINOR=${_hpm#GPIB minor=}
    case "$EMU_GPIB_MINOR" in
        [0-9] | 1[0-4]) ;;
        *) echo "hp-3478a: gpib-config said '$_hpm'" >&2; return 1 ;;
    esac
    [ -c "/dev/gpib$EMU_GPIB_MINOR" ] || { echo "hp-3478a: no /dev/gpib$EMU_GPIB_MINOR" >&2; return 1; }
}
# At the session's end the board goes offline and its node's mode back.
emu_detaching() {
    sudo -n "$EMU" gpib-config --release >/dev/null
}

# The kernel keeps its interrupt monitor across cases: whether a case's first status
# read is followed by a re-arm (WAIT_REQUEST 0x300) depends on how the case before
# it ended, not on this one. That one line goes from the head of every .tx.
MON='1,2{/^ctl_in 40 21 0300 /d}'
hp() {
    _hpn=$1
    shift
    emu_case "$_hpn" --no-oracle-only --tx-filter "$MON" "$@"
}

emu_cases() {
    D=hp-3478a:conn=gpib/${EMU_GPIB_MINOR:-0}/23
    hp "scan" --keep-header -- -d $D --scan
    hp "show" --keep-header -- -d $D --show
    hp "get measured_quantity" -- -d $D --get measured_quantity
    hp "get range" -- -d $D --get range
    hp "get digits" -- -d $D --get digits
    hp "scan with no conn" --no-output -- -d hp-3478a --scan
    hp "scan libgpib names" --stderr -- -d hp-3478a:conn=libgpib/hp3478a --scan
    hp "scan no such board" --no-output -- -d hp-3478a:conn=gpib/15/23 --scan
    hp "capture" -- -d $D --samples 3
    hp "capture analog" -- -d $D --samples 2 -O analog

    # Each function, by its setting, and a sample of it.
    for _mq in voltage/dc voltage/ac current/dc current/ac resistance resistance/four_wire; do
        hp "mq $_mq" -- -d $D -c measured_quantity=$_mq --samples 1
    done
    hp "mq the same sends nothing" --no-output -- -d $D -c measured_quantity=voltage/dc --set
    hp "mq with no ac or dc" --stderr --expect-fail -- -d $D -c measured_quantity=voltage --set
    hp "mq four wire after two" -- -d $D -c measured_quantity=resistance -c measured_quantity=resistance/four_wire --samples 1

    # Each range of each function, set from what the meter shows.
    for _r in Auto 30mV 300mV 3V 30V 300V; do
        hp "dcv range $_r" -- -d $D -c range=$_r --samples 1
    done
    emu_options init=F2
    for _r in 300mV 3V 30V 300V; do
        hp "acv range $_r" -- -d $D -c range=$_r --samples 1
    done
    hp "acv has no 30mV" --stderr --expect-fail -- -d $D -c range=30mV --set
    emu_options init=F5R0
    for _r in Auto 300mA 3A; do
        hp "dca range $_r" -- -d $D -c range=$_r --samples 1
    done
    emu_options init=F6
    hp "aca range 300mA" -- -d $D -c range=300mA --samples 1
    emu_options init=F3
    for _r in 30R 300R 3kR 30kR 300kR 3MR 30MR; do
        hp "ohm range $_r" -- -d $D -c range=$_r --samples 1
    done
    hp "ohm show" --keep-header -- -d $D --show
    emu_options init=F4R2
    hp "four wire 300R" -- -d $D --samples 1
    emu_options init=F7
    hp "extended ohms" -- -d $D --samples 1
    emu_options

    for _d in 3.5 4.5 5.5; do
        hp "digits $_d" -- -d $D -c digits=$_d --samples 2
    done
    hp "digits 6.5" --stderr --expect-fail -- -d $D -c digits=6.5 --set

    # The status bytes the meter's switches set.
    emu_options rear=1 hz=50 cal=1
    hp "rear 50 Hz cal" -- -d $D --samples 1
    emu_options power-on-srq=1
    hp "power-on srq" -- -d $D --samples 2
    emu_options every=1
    hp "a reading at each poll" -- -d $D --samples 3
    emu_options every=4
    hp "a reading every fourth poll" -- -d $D --samples 2

    # A meter that is not there, or not well.
    emu_options pad=9
    hp "nothing at the address" --no-output -- -d $D --scan
    emu_options absent=1
    hp "nothing on the bus" --no-output -- -d $D --scan
    emu_options error=08
    hp "error register set" --no-output -- -d $D --scan
    emu_options short-status=3
    hp "status of three bytes" --no-output -- -d $D --scan
    emu_options mute-status=1
    hp "status not answered" --no-output -- -d $D --scan
    emu_options mute-status=4
    hp "status not answered in a capture" --oxrok-fails "timeout" -- -d $D --samples 3
    emu_options mute-poll=3
    # The model answers at once with the adapter's timeout, where the kernel's serial
    # poll of a real mute meter would give its timeout itself: either words pass.
    hp "poll not answered" --oxrok-fails "timeout|no answer" -- -d $D --samples 3
    emu_options mute-read=2
    hp "reading not answered" --oxrok-fails "timeout" -- -d $D --samples 3
    emu_options cut-read=2
    hp "reading cut short" --oxrok-fails "timeout" -- -d $D --samples 3
    # A board a program before left in an end of string mode on 0x17, the binary
    # status's second byte: a read that kept it would end the status there. oxrok
    # clears it before each read, as libgpib's ibrd does.
    [ "$EMU_DRY" = 1 ] || sudo -n "$EMU" gpib-config --minor "${EMU_GPIB_MINOR:-0}" --eos 23 \
        || echo "hp-3478a: could not leave the board in an end of string mode" >&2
    emu_options
    hp "a board left in an eos mode" -- -d $D --samples 2
    emu_options junk-at=2
    hp "reading no number" --oxrok-fails "no number" -- -d $D --samples 3
    emu_options overload-at=2
    hp "overload" -- -d $D --samples 3
    emu_options

    # Gone mid-capture, and stopped.
    emu_options mark-at=2
    hp "unplug" --unplug model --oxrok-fails "no adapter" -- -d $D --samples 5
    # SIGINT lands when it lands: what the capture did between it and the stop (a
    # poll, a reading more) goes, up to the T1 of the end, and the output keeps the
    # two readings the mark guarantees.
    INT_TX='/^interrupt$/,/^bulk_out 02 0dfdfffc0000080054310a/{/^interrupt$/b;/54310a/b;d}'
    hp "interrupt" --interrupt model --tx-filter "$MON
$INT_TX" --filter '3,$d' -- -d $D --continuous
    emu_options
}
