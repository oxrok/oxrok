# The Intronix LogicPort LA1034 (emu/src/models/intronix-logicport.rs): an
# FT245BL in front of a Cyclone FPGA the host configures in bit-bang mode.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a device
# model". The oracle is the C driver of libsigrok PR 315, which main's
# oracle has once its patch is folded in; until then a run sets
# EMU_ORACLE_ENV and EMU_LIBSIGROK (tools/diffcli.sh).
#
# The C driver has patterns and edges only, so what is oxrok's alone
# (counts, durations, ranges, or, and, qualifier, sample point, inversion,
# the frequency counter) is in --no-oracle-only cases, whose goldens hold
# the bench's criteria as well as they can: each says which check of
# hil/oxrok-hil it is, and the model's unit tests apply the same criteria
# to its samples.
EMU_MODEL=intronix-logicport
EMU_OPTIONS=""
EMU_DRIVER=intronix-logicport
EMU_GCOV_SHARED="src/usb.c"
EMU_FULL=intronix-logicport
EMU_TX_FOLD=1

# The model's options for the cases that follow.
lp_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "intronix-logicport: the emulator refused '$*'"
}

# A stand-in for the vendor's LogicPort.ccf in directory $1: the file's real
# form (header, images as bit-bang streams with their first two blocks
# swapped), with made-up images of the sizes standin_size() in the model
# gives. $2 says what to spoil, for the fault cases. No vendor bytes.
lp_ccf() {
    python3 -I - "$1" "${2:-}" <<'PY'
import hashlib, os, sys
out, spoil = sys.argv[1], sys.argv[2]
names = ["state-after", "state-before-clk1-rise", "state-before-clk2-rise",
         "state-before-clk1-fall", "state-before-clk2-fall", "state-on",
         "timing-ddr", "timing-sdr"]
KEY = 0x916662
images, units = [], []
for i, name in enumerate(names):
    size = 3000 + 16 * i
    rbf = bytearray(b"\xff" * 16 + b"\x6a")
    block = 0
    while len(rbf) < size:
        rbf += hashlib.sha256(f"intronix-logicport-{name}:{block}".encode()).digest()
        block += 1
    del rbf[size:]
    rbf[-1] = 0xff
    if spoil == "image":      # a block out of place, past the RBF check
        rbf[1000], rbf[2000] = rbf[2000], rbf[1000]
    stream = bytearray()
    for b in rbf:
        for k in range(8):
            bit = b >> k & 1
            stream += bytes([0x04 | bit, 0x06 | bit])
    stream[0:4096], stream[4096:8192] = stream[4096:8192], stream[0:4096]
    if spoil == "rbf":        # blocks not swapped: no RBF sync
        stream[0:4096], stream[4096:8192] = stream[4096:8192], stream[0:4096]
    images.append(bytes(stream))
    units.append(size)
check = 0
for u in units:
    check ^= u
fields = [u ^ KEY for u in units] + [check ^ KEY]
if spoil == "check":
    fields[8] ^= 1
head = ",".join(map(str, fields)).encode() + b"\r\n"
data = head + b"".join(images)
if spoil == "short":
    data = data[:-1]
if spoil == "text":
    data = b"not a ccf file\r\n" + b"x" * 100
os.makedirs(out, exist_ok=True)
dest = os.path.join(out, "intronix-logicport.ccf")
try:
    with open(dest, "rb") as f:
        if f.read() == data:
            sys.exit(0)
except FileNotFoundError:
    pass
with open(dest + ".tmp", "wb") as f:
    f.write(data)
os.replace(dest + ".tmp", dest)
PY
}

emu_cases() {
    D=intronix-logicport
    lp_ccf "$EMU_FIRMWARE_DIR"
    FW=$EMU_FIRMWARE_DIR
    # The USB level differences between libftdi and oxrok's FTDI layer, from
    # each tool's log. None of them reaches the FPGA; docs/DEVICES.md lists
    # them, and tests/logicport_replay.rs holds oxrok's to the real unit's
    # recordings: libftdi's string reads at the scan and its reset and 9600 Bd
    # at each open; its port 1 in wIndex, where oxrok sends 0; the second half
    # of a 128 word EEPROM read of a 64 word part; the order of its two purges.
    T='/^ctl_in 00 06 /d
/^ctl_in 40 90 0000 00[4-7][0-9a-f] /d
/^ctl_out 40 00 0000 0001 /{$!N;/\nctl_out 40 03 4138 /d;}
/^ctl_out 40 00 0001 0001 /{$!N;s/^([^\n]*)\n(ctl_out 40 00 0002 0001 [^\n]*)$/\2\n\1/;}
/^ctl_in 40 90 /!s/^(ctl_(in|out) 40 [0-9a-f]{2} [0-9a-f]{4}) 0001 /\1 0000 /Mg'
    # After an upload the C driver's init wait is by the clock, oxrok's by
    # reads, so how many reads of the counter they make differs and the
    # sequence numbers with it: those go, and a run of the same line counts
    # once.
    U="$T"'
/^bulk_out 02 c205000000 /{N;d;}
/^bulk_in 81 [0-9]+ -> 3160$/d
s/^(bulk_in 81 [0-9]+ -> )@([0-9]+):.*$/\1@\2:READOUT/
s/^(bulk_in 81 [0-9]+ -> 3160c[123])[0-9a-f]{4}/\1SEQ/'
    # A failure's wait is by the clock: how many empty reads a timeout saw.
    S="$T"'
/^bulk_in 81 [0-9]+ -> 3160$/d'
    # What a capture says when it fails is the driver's own in each tool: the
    # C driver's generic lines and sigrok-cli's exit status against oxrok's
    # error and exit 1. These cases compare the transactions and that nothing
    # comes out; the "oxrok ... words" cases at the end pin oxrok's.
    # A wait on a pin is by the clock too.
    W="$U"'
/^ctl_in 40 0c /d'
    # The tools' log lines as sigrok-cli's, and what only one of them says
    # around a refused start.
    L='s/^oxrok: [a-z]+: oxrok::drivers::logicport(::[a-z]+)?: /sr: intronix-logicport: /
/^sr: session: /d'

    emu_case "scan" --keep-header --tx-filter "$T" -- -d $D --scan
    emu_case "show" --keep-header --tx-filter "$T" --filter 's/conn: [0-9]+\.[0-9]+/conn: BUS.ADDR/' --oracle-differs "oxrok's show has the extra options and measures the frequency" -- -d $D --show
    for k in samplerate limit_samples limit_frames captureratio voltage_threshold rle external_clock external_clock_source clock_edge; do
        emu_case "get $k" --tx-filter "$T" -- -d $D --get $k
    done
    emu_case "get conn" --tx-filter "$T" --filter 's/[0-9]+\.[0-9]+/BUS.ADDR/' -- -d $D --get conn
    emu_case "select by conn" --tx-filter "$T" --keep-header -- -d $D:conn=$EMU_USB --scan

    # Every rate of the table: its prescaler and step in the .tx, the clock
    # in the samples.
    for r in 1k 2k 5k 10k 20k 50k 100k 200k 500k 1M 2M 5M 10M 20M 50M 100M 200M 250M; do
        emu_case "rate $r" --tx-filter "$T" -- -d $D -c samplerate=$r -C CLK1 --samples 64 -O bits
    done
    emu_case "whole memory" --tx-filter "$T" --digest --filter '12,$d' -- -d $D -c samplerate=100M -C D19,D21,CLK1 --samples 2048 -O bits
    emu_case "all channels 40 samples" --tx-filter "$T" -- -d $D -c samplerate=10M --samples 40 -O hex
    emu_case "csv with trigger" --tx-filter "$T" -- -d $D -c samplerate=10M:captureratio=25 -C D0,CLK1 --samples 200 -t CLK1=r -O csv:trigger=true:header=false
    for ratio in 0 1 25 50 99 100; do
        emu_case "ratio $ratio" --tx-filter "$T" -- -d $D -c samplerate=10M:captureratio=$ratio -C CLK1 --samples 100 -t CLK1=r -O bits
    done
    emu_case "more samples than memory" --tx-filter "$T" --digest --filter '4,$d' -- -d $D -c samplerate=10M -C CLK1 --samples 5000 -O bits
    emu_case "limit in time" --tx-filter "$T" --digest --filter '4,$d' -- -d $D -c samplerate=1M -C CLK1 --time 1 -O bits
    for th in 1.5 0.0 m6.0 6.0 3.25; do
        v=$(echo $th | tr m -)
        emu_case "threshold $th" --tx-filter "$T" -- -d $D -c samplerate=1M:voltage_threshold=$v-$v -C D19,CLK1 --samples 16 -O bits
    done

    # Triggers: the forms the C driver has.
    for t in CLK1=r CLK1=f CLK1=e CLK1=0 CLK1=1 D19=1 'CLK1=r,D19=1' 'CLK1=f,D19=1'; do
        emu_case "trigger $t" --tx-filter "$T" -- -d $D -c samplerate=10M -C D0,D19,CLK1 --samples 64 -t "$t" -O bits
    done
    emu_case "trigger 500M" --tx-filter "$T" --digest --filter '6,$d' -- -d $D -c samplerate=500M -C CLK1 --samples 2047 -t CLK1=r -O bits
    emu_case "trigger 500M ratio 25" --tx-filter "$T" --digest --filter '6,$d' -- -d $D -c samplerate=500M:captureratio=25 -C CLK1 --samples 2047 -t CLK1=r -O bits
    emu_case "trigger on a channel left out" --tx-filter "$T" --stderr --expect-fail --filter "$L" -- -d $D -c samplerate=1M -C CLK1 --samples 16 -t D0=r -O bits
    # Bench stimuli, in time: the D0 pulse train of the "trigger" section.
    lp_options d=0 seq=1:5000,0:5000 seq-repeat=30 clk=0
    emu_case "pulses rising 1M" --tx-filter "$T" -- -d $D -c samplerate=1M:captureratio=25 -C D0 --samples 200 -t D0=r -O bits
    emu_case "pulses falling 1M" --tx-filter "$T" -- -d $D -c samplerate=1M -C D0 --samples 100 -t D0=f -O bits
    # What the user ran on a real unit, from the macOS release binary.
    emu_case "wait for the trigger" --tx-filter "$T" --oracle-differs "sigrok-cli session.c:403: -w keeps samples from the decoder and the count only, and its output module gets them all; oxrok's output starts at the trigger" -- -d $D -c samplerate=1M -C D0 --samples 100 -t D0=r -w -O bits
    emu_case "wait for the trigger, whole memory" --tx-filter "$T" --oracle-differs "sigrok-cli session.c:403: -w keeps samples from the decoder and the count only, and its output module gets them all; oxrok's output starts at the trigger" --digest --filter '12,$d' -- -d $D -c samplerate=1M --samples 2048 -t D0=r -w -O bits
    lp_options d=0 seq=0:10000,1:10000,2:10000,3:10000,4:10000,5:10000,6:10000,7:10000 seq-repeat=3 clk=0
    emu_case "pattern 110 1M" --tx-filter "$T" -- -d $D -c samplerate=1M -C D0,D1,D2 --samples 100 -t D0=0,D1=1,D2=1 -O bits
    # The upper lanes: D24 rising while D30 is high.
    lp_options d=0 seq=0:20000,0x1000000:10000,0:10000,0x40000000:10000,0x41000000:10000,0x1000000:10000,0:10000 clk=0
    emu_case "upper lanes 1M" --tx-filter "$T" -- -d $D -c samplerate=1M -C D24,D30 --samples 100 -t D30=1,D24=r -O bits
    # Compression. The drivers' time limits have run out at the first poll
    # at these sizes.
    lp_options clk=100000
    emu_case "recording 1M compressed" --tx-filter "$T" --record -- -d $D -c samplerate=1M:rle=on:voltage_threshold=1.5-1.5 -C CLK1 --samples 500 -t CLK1=r -O bits
    lp_options
    emu_case "rle more than memory" --tx-filter "$T" --digest --filter '4,$d' -- -d $D -c samplerate=1M:rle=on -C CLK1 --samples 4000 -O bits
    emu_case "rle above 200M refused" --tx-filter "$T" --stderr --expect-fail --filter "$L
/^Failed to start session/d
s/^sr: intronix-logicport: RLE/RLE/" -- -d $D -c samplerate=250M:rle=on -C CLK1 --samples 100 -O bits
    lp_options d=0 seq=0:100000,1:100000,2:100000,3:100000,4:100000,5:100000,6:100000,7:100000 seq-repeat=2 clk=0
    emu_case "rle 100 us steps" --tx-filter "$T" --digest --filter '6,$d' -- -d $D -c samplerate=1M:rle=on -C D0,D1,D2 --samples 1200 -t D0=1,D1=1,D2=1 -O bits
    lp_options d=0 clk=0
    emu_case "rle idle inputs" --tx-filter "$T" --digest --filter '4,$d' -- -d $D -c samplerate=1M:rle=on -C D0 --samples 500 -O bits
    lp_options

    # State mode: the bench's 100 kHz on CLK1, CLK2 at 1 MHz.
    lp_options clk=100000 clk2=1000000 image=5
    for e in rising falling; do
        for c in CLK1 CLK2; do
            emu_case "state $c $e" --tx-filter "$T" -- -d $D -c external_clock=on:external_clock_source=$c:clock_edge=$e -C D19,CLK1,CLK2 --samples 32 -O bits
        done
    done
    lp_options

    # Frames: each capture its own, the inputs going on.
    emu_case "three frames" --tx-filter "$T" -- -d $D -c samplerate=10M -C CLK1 --samples 32 --frames 3 -t CLK1=r -O bits
    lp_options fill=5
    emu_case "fill polls" --tx-filter "$T" -- -d $D -c samplerate=10M -C CLK1 --samples 16 -O bits

    # The FTDI and EEPROM. A calibration moves the DAC code in the .tx.
    lp_options cal=+003
    emu_case "calibration +003" --tx-filter "$T" -- -d $D -c samplerate=1M -C CLK1 --samples 8 -O bits
    lp_options cal=none
    emu_case "no calibration" --tx-filter "$T" -- -d $D -c samplerate=1M -C CLK1 --samples 8 -O bits
    lp_options stall=read-eeprom
    emu_case "eeprom stalls" --tx-filter "$T" -- -d $D -c samplerate=1M -C CLK1 --samples 8 -O bits
    lp_options

    # Settings refused, each before anything goes out; the words differ.
    for c in samplerate=123 captureratio=101 voltage_threshold=9.0-9.0 external_clock_source=CLK3 clock_edge=up; do
        emu_case "refused $c" --tx-filter "$T" --no-output --expect-fail -- -d $D -c $c --set
    done
    # The FTDI refuses a setup request: the open fails.
    for s in reset latency baud/0x4006; do
        r=${s%/*}
        lp_options stall=$s
        emu_case "open with $r stalled" --tx-filter "$S" --no-output --expect-fail -- -d $D --get samplerate
    done
    lp_options

    # The FPGA and its upload.
    lp_options image=none
    emu_case "cold upload" --tx-filter "$U" --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 32 -O bits
    lp_options image=6
    emu_case "reconfiguration from 500 MHz" --tx-filter "$U" --timeout 20 -- -d $D -c samplerate=10M:voltage_threshold=1.5-1.5 -C D0,D1,D2,CLK1 --samples 64 -t CLK1=r -O bits
    lp_options image=7
    emu_case "to the 500 MHz image" --tx-filter "$U" --digest --filter '4,$d' --timeout 20 -- -d $D -c samplerate=500M -C CLK1 --samples 2047 -O bits
    lp_options image=none stale=0
    emu_case "upload with no stale bytes" --tx-filter "$U" --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 32 -O bits
    lp_options image=none init-steps=8
    emu_case "init counter counts" --tx-filter "$U" --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none poweron-silent=3
    emu_case "ignores commands after power on" --tx-filter "$U" --timeout 30 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options

    lp_options image=none id=0x42
    emu_case "unexpected image id" --tx-filter "$U" --no-output --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none wrong-image=6
    emu_case "the wrong image loads" --tx-filter "$U" --no-output --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none nstatus-stuck=1
    emu_case "nstatus stays low" --tx-filter "$W" --oracle-bug "protocol.c:380-399: libftdi keeps its bit-bang flag across a close, so a reopen after a failed upload asks 4 x 460800 Bd, which it refuses, and every retry fails" --stderr --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none conf-done-stuck=1
    emu_case "conf_done low once, the retry works" --tx-filter "$W" --oracle-bug "protocol.c:380-399: libftdi keeps its bit-bang flag across a close, so a reopen after a failed upload asks 4 x 460800 Bd, which it refuses, and every retry fails" --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none conf-done-stuck=100
    emu_case "conf_done stays low" --tx-filter "$W" --oracle-bug "protocol.c:380-399: libftdi keeps its bit-bang flag across a close, so a reopen after a failed upload asks 4 x 460800 Bd, which it refuses, and every retry fails" --stderr --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none init-stuck=1
    emu_case "init counter stuck, and both go on" --tx-filter "$U" --timeout 50 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options

    # A link that stops or lies, past the upload.
    lp_options silent-reg=0x100000
    emu_case "status silent" --tx-filter "$S" --oxrok-fails "timeout" --no-output --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options silent-reg=0x100002
    emu_case "arm silent" --tx-filter "$S" --no-output --expect-fail --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options short=3
    emu_case "a lane read stops half way" --tx-filter "$S" --oxrok-fails "timeout" --no-output --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options bad-seq=20
    emu_case "a bad ack in the settings" --tx-filter "$T" --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options bad-seq=54
    emu_case "a bad ack in the readout" --tx-filter "$S" --oxrok-fails "bad ack" --no-output --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options in-stall=6
    # Up to the halt: what follows is a wait_link's retries, by the clock.
    emu_case "bulk in halts" --tx-filter "$S
/err stall\$/q" --no-output --expect-fail --timeout 30 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options stall=bitmode image=none
    emu_case "bit mode stalls" --tx-filter "$W
/^ctl_out 40 0b 0000 0000 - -> err stall/d" --no-output --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options
    emu_case "unplug at the status" --tx-filter "$S" --oxrok-fails "disconnected" --no-output --unplug in=1:46 --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    emu_case "unplug in the readout" --tx-filter "$S" --oxrok-fails "disconnected" --no-output --unplug in=1:50 --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options d=0 clk=0 silent-reg=0x10000a
    emu_case "end of pre-fill silent" --tx-filter "$S" --oxrok-fails "timeout" --no-output --timeout 20 -- -d $D -c samplerate=1M:rle=on -C D0 --samples 500 -O bits
    lp_options silent-after=60
    emu_case "no answer to the next frame" --tx-filter "$S" --oxrok-fails "timeout" --timeout 20 -- -d $D -c samplerate=10M -C CLK1 --samples 16 --frames 2 -O bits
    lp_options d=0 clk=0
    emu_case "a trigger that never comes, interrupted" --oracle-bug "sigrok-cli has no SIGINT handler" --tx-filter "$T
/^interrupt\$/q" --interrupt in=1:70 --no-output --timeout 10 -- -d $D -c samplerate=1M -C D0 --samples 16 -t D0=r -O bits
    lp_options

    # Firmware.
    M=$TMP/lp-fw
    lp_options image=none
    SIGROK_FIRMWARE_DIR=$M/missing; export SIGROK_FIRMWARE_DIR
    mkdir -p "$M/missing"
    emu_case "no firmware" --tx-filter "$U" --no-output --expect-fail --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    for spoil in text check short rbf image; do
        lp_ccf "$M/$spoil" $spoil
        SIGROK_FIRMWARE_DIR=$M/$spoil
        if [ $spoil = image ]; then
            emu_case "firmware $spoil" --tx-filter "$W" --oracle-bug "the bit-bang flag, as above" --stderr --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
        else
            emu_case "firmware $spoil" --tx-filter "$U" --no-output --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
        fi
    done
    SIGROK_FIRMWARE_DIR=$FW
    lp_options
    # What only oxrok's driver has: sigrok-cli takes neither the -t forms nor
    # the options. Each pins a check of hil/oxrok-hil, named; the model's unit
    # tests hold its samples to the criterion itself.
    O="--no-oracle-only --tx-filter"
    # count: the Nth rise of the 10 us train.
    lp_options d=0 seq=1:5000,0:5000 seq-repeat=30 clk=0
    for n in 1 5 17; do
        emu_case "oxrok count r*$n" $O "$T" -- -d $D -c samplerate=1M -C D0 --samples 200 -t "D0=r*$n" -O bits
    done
    # A level with a count: the pattern becoming true, not its samples.
    emu_case "oxrok count of a level" $O "$T" -- -d $D -c samplerate=1M -C D0 --samples 200 -t "D0=1*2" -O bits
    # consecutive: six rises, D1 high except on the third.
    lp_options d=0 seq=0:20000,2:5000,3:5000,2:5000,3:5000,0:5000,1:5000,2:5000,3:5000,2:5000,3:5000,2:5000,3:5000,0:20000 clk=0
    emu_case "oxrok cumulative r*3" $O "$T" -- -d $D -c samplerate=1M -C D0,D1 --samples 200 -t "D0=r*3,D1=1" -O bits
    emu_case "oxrok consecutive r*3c" $O "$T" -- -d $D -c samplerate=1M -C D0,D1 --samples 200 -t "D0=r*3c,D1=1" -O bits
    # duration: five 50 us pulses, then one of 200 us.
    lp_options d=0 seq=0:20000,1:50000,0:50000,1:50000,0:50000,1:50000,0:50000,1:50000,0:50000,1:50000,0:50000,1:200000,0:50000 clk=0
    emu_case "oxrok duration 100 samples" $O "$T" -- -d $D -c samplerate=1M -C D0 --samples 400 -t "D0=1>100" -O bits
    emu_case "oxrok duration 150 us" $O "$T" -- -d $D -c samplerate=1M -C D0 --samples 400 -t "D0=1>150us" -O bits
    # value: D2..D0 count 0..7 in 10 us steps.
    lp_options d=0 seq=0:10000,1:10000,2:10000,3:10000,4:10000,5:10000,6:10000,7:10000 seq-repeat=3 clk=0
    emu_case "oxrok value inside" $O "$T" -- -d $D -c samplerate=1M -C D0,D1,D2 --samples 100 -t "D0-D2=3..5" -O bits
    emu_case "oxrok value outside" $O "$T" -- -d $D -c samplerate=1M -C D0,D1,D2 --samples 100 -t "D0-D2=!0..6" -O bits
    # combine.
    lp_options d=0 seq=0:20000,2:5000,0:5000,2:5000,0:5000,2:5000,0:20000 clk=0
    emu_case "oxrok or on D1 alone" $O "$T" -- -d $D -c samplerate=1M -C D0,D1 --samples 100 -t "D0=r|D1=r" -O bits
    lp_options d=0 seq=0:20000,2:5000,0:5000,2:5000,0:20000,1:5000,0:5000,1:5000,0:20000 clk=0
    emu_case "oxrok and D1 then D0" $O "$T" -- -d $D -c samplerate=1M -C D0,D1 --samples 100 -t "D0=r&D1=r" -O bits
    lp_options d=0 seq=0:20000,2:5000,0:5000,2:5000,0:5000,1:5000,0:5000,2:5000,0:5000,2:5000,0:5000 clk=0
    emu_case "oxrok two stages" $O "$T" -- -d $D -c samplerate=1M -C D0,D1 --samples 100 -t "D0=r;D1=r" -O bits
    emu_case "oxrok three stages refused" $O "$T" --stderr --expect-fail -- -d $D -c samplerate=1M -C D0 --samples 16 -t "D0=r;D0=f;D0=r" -O bits
    # invert: "D0 reads inverted"; "D0=r: marker on a rising edge of the inverted data".
    lp_options d=1 clk=0
    emu_case "oxrok invert high reads low" $O "$T" -- -d $D -c samplerate=1M:invert=D0 -C D0 --samples 16 -O bits
    lp_options d=0 seq=1:5000,0:5000 seq-repeat=30 clk=0
    emu_case "oxrok invert trigger" $O "$T" -- -d $D -c samplerate=1M:invert=D0 -C D0 --samples 100 -t D0=r -O bits
    # qualifier: a capture with the qualifier false never completes; CLK2 here.
    lp_options clk=100000 d=0 image=5
    emu_case "oxrok qualifier off" $O "$T" -- -d $D -c external_clock=on:clock_qualifier=off -C CLK1,CLK2 --samples 16 -O bits
    lp_options clk=100000 d=0 image=5 clk2=1
    emu_case "oxrok qualifier active-low" $O "$T" -- -d $D -c external_clock=on:clock_qualifier=active-low -C CLK1,CLK2 --samples 16 -O bits
    lp_options clk=100000 d=0 image=5
    emu_case "oxrok qualifier false never completes, interrupted" $O "$T
/^interrupt\$/q" --interrupt in=1:70 --no-output --timeout 10 -- -d $D -c external_clock=on:clock_qualifier=active-high -C CLK1,CLK2 --samples 16 -O bits
    # sample point: a change of image, and a clock inverted makes the edge the other one.
    for pe in "before rising" "before falling" "after rising"; do
        set -- $pe
        lp_options clk=100000 d=0 image=5
        emu_case "oxrok sample point $1 $2" $O "$U" --timeout 20 -- -d $D -c external_clock=on:sample_point=$1:clock_edge=$2 -C CLK1 --samples 16 -O bits
    done
    lp_options clk=100000 d=0 image=5
    emu_case "oxrok sample point before inverted clock" $O "$U" --timeout 20 -- -d $D -c external_clock=on:sample_point=before:invert=CLK1 -C CLK1 --samples 16 -O bits
    # freq: the input's frequency to 10 Hz, the second reading after a select.
    lp_options image=none clk=1000000
    emu_case "oxrok show on a cold device uploads" $O "$U" --keep-header --filter 's/conn: [0-9]+\.[0-9]+/conn: BUS.ADDR/' --timeout 20 -- -d $D --show
    lp_options clk=1000000
    emu_case "oxrok frequency on CLK1" $O "$T" -- -d $D -c frequency_channel=CLK1 --get frequency
    emu_case "oxrok frequency on D0" $O "$T" -- -d $D -c frequency_channel=D0 --get frequency
    lp_options
    # The words of a failure, which the full cases map.
    lp_options silent-reg=0x100000
    emu_case "oxrok status silent words" $O "$S" --stderr --oxrok-fails "timeout" --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options
    SIGROK_FIRMWARE_DIR=$M/missing
    lp_options image=none
    emu_case "oxrok no firmware words" $O "$U" --stderr --expect-fail --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    for spoil in text short rbf; do
        SIGROK_FIRMWARE_DIR=$M/$spoil
        emu_case "oxrok firmware $spoil words" $O "$U" --stderr --expect-fail --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    done
    SIGROK_FIRMWARE_DIR=$FW
    lp_options silent-reg=0x100002
    emu_case "oxrok arm silent words" $O "$S" --stderr --expect-fail --timeout 20 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options image=none id=0x42
    emu_case "oxrok unexpected image id words" $O "$U" --stderr --expect-fail --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 16 -O bits
    lp_options
    # The vendor's own file, when a run has it (never in CI or the goldens):
    # EMU_LP_REAL_FW names its directory. No golden, so the two tools are held
    # to each other alone.
    if [ -n "${EMU_LP_REAL_FW:-}" ] && [ "${EMU_BLESS:-0}" != 1 ] && [ $no_oracle = 0 ]; then
        SIGROK_FIRMWARE_DIR=$EMU_LP_REAL_FW
        lp_options image=none
        emu_case "real file cold upload" --tx-filter "$U" --timeout 40 -- -d $D -c samplerate=1M -C CLK1 --samples 32 -O bits
        lp_options image=7
        emu_case "real file to 500M" --tx-filter "$U" --digest --filter '4,$d' --timeout 40 -- -d $D -c samplerate=500M -C CLK1 --samples 2047 -O bits
        SIGROK_FIRMWARE_DIR=$FW
        lp_options
    fi
}
