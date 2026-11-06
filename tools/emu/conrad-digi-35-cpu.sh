# A Conrad DIGI 35 CPU (emu/src/models/conrad-digi-35-cpu.rs) on the CDC-ACM
# shell. It takes commands and never answers, so each case's `.tx` is what a case
# pins. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial
# model".
EMU_MODEL=conrad-digi-35-cpu
EMU_CDC=1
EMU_DRIVER=conrad-digi-35-cpu
EMU_FULL=$EMU_DRIVER
EMU_GCOV=conrad-digi-35-cpu
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only. A takes off exactly what oxrok's error
    # adds to the core's wording, so another text fails, and "refusal says so" pins
    # the words themselves.
    L="s/^oxrok: err: oxrok::drivers::conrad_digi_35_cpu(::protocol)?: /sr: conrad-digi-35-cpu: /"
    A="s/^(Failed to set device option '([a-z_]+)': invalid argument): \\2 out of range\\.$/\\1./"
    R="$L
$A"
    QUEUE_BUG="protocol.c:46-47 returns success on a frame the tty took, so C exits 0 on a setting still in the queue, and fails only when the queue is full; oxrok waits for the queue before the set returns"
    HASH_ORDER="sigrok-cli sets one -c in its hash table's order (session.c:700-701), here current first; oxrok in the order given"
    NAN_BUG="api.c:105 and :112's two comparisons let a NaN through, :109 and :116 convert it to INT_MIN on x86-64, and protocol.c:41 cuts that to V-21 or C-21, sent to a live output; oxrok refuses it"

    # The scan can only open the port and assume the device is there.
    emu_options
    emu_case "scan" --keep-header -- -d $C --scan
    emu_case "show" --keep-header -- -d $C --show
    emu_case "no conn" --keep-header --no-output -- -d conrad-digi-35-cpu --scan
    emu_case "no conn stderr" --keep-header --stderr --expect-fail -- -d conrad-digi-35-cpu --show
    emu_case "no such node" --keep-header --no-output -- -d conrad-digi-35-cpu:conn=/dev/ttyACM97 --scan
    emu_case "serialcomm as it is" --keep-header -- -d $C:serialcomm=9600/8n1 --scan
    emu_case "serialcomm no such" --tx-filter '/^ctl_out/d' --keep-header --no-output -- -d $C:serialcomm=fast --scan
    # A wrong rate can't be noticed: no answer is missing, the set succeeds.
    emu_case "serialcomm wrong and no one knows" --keep-header --no-output -- -d $C:serialcomm=4800/8n1 --set -c voltage_target=5
    emu_options uart=4800/8n1
    emu_case "serialcomm kept by the open" --keep-header --no-output -- -d $C:serialcomm=4800/8n1 --set -c voltage_target=5
    emu_options

    # Nothing can be read.
    emu_case "get voltage" --keep-header --stderr --expect-fail -- -d $C --get voltage_target
    emu_case "get conn" --keep-header --stderr --expect-fail -- -d $C --get conn
    emu_case "get after set" --keep-header --stderr --expect-fail -- -d $C -c voltage_target=5 --get voltage_target
    emu_case "no such group" --keep-header --stderr --expect-fail -- -d $C -g CH1 --show

    # The conversions, in sigrok-cli's single precision.
    for v in 0 5 12.34 35 3.3 1.15 0.04 0.05 34.96; do
        emu_case "set voltage $v" --keep-header --no-output -- -d $C --set -c voltage_target=$v
    done
    for v in 0 1 2.55 1.005 0.29 0.004 0.005; do
        emu_case "set current $v" --keep-header --no-output -- -d $C --set -c current_limit=$v
    done
    emu_case "set voltage minus 0" --keep-header --no-output -- -d $C --set -c voltage_target=-0
    emu_case "set current minus 0" --keep-header --no-output -- -d $C --set -c current_limit=-0
    emu_case "set protection on" --keep-header --no-output -- -d $C --set -c ocp_enabled=on
    emu_case "set protection off" --keep-header --no-output -- -d $C --set -c ocp_enabled=off

    # Refused by the driver, nothing on the wire.
    for v in 35.1 -1 inf; do
        emu_case "refuse voltage $v" --keep-header --stderr --expect-fail --filter "$R" -- -d $C --set -c voltage_target=$v
    done
    for v in 2.56 -0.01 inf; do
        emu_case "refuse current $v" --keep-header --stderr --expect-fail --filter "$R" -- -d $C --set -c current_limit=$v
    done
    emu_case "set voltage nan" --oracle-differs "$NAN_BUG" --keep-header --stderr --expect-fail --filter "$R" -- -d $C --set -c voltage_target=nan
    emu_case "set current nan" --oracle-differs "$NAN_BUG" --keep-header --stderr --expect-fail --filter "$R" -- -d $C --set -c current_limit=nan
    # The differs cases hold oxrok's .tx and refusal, so C's frames are pinned here:
    # with no limit it goes on to the start and the session refuses.
    emu_case "lib nan goes out" --libsigrok --keep-header --stderr --expect-fail -- -d $C -c voltage_target=nan -c current_limit=nan

    # Each -c in turn, and what a refusal leaves behind.
    emu_case "set two" --keep-header --no-output -- -d $C --set -c voltage_target=5 -c current_limit=1
    emu_case "set two reversed" --keep-header --no-output -- -d $C --set -c current_limit=1 -c voltage_target=5
    emu_case "set current then voltage in one" --keep-header --no-output -- -d $C --set -c current_limit=1:voltage_target=5
    emu_case "set voltage then current in one" --oracle-differs "$HASH_ORDER" --keep-header --no-output -- -d $C --set -c voltage_target=5:current_limit=1
    emu_case "set then refused" --keep-header --stderr --expect-fail --filter "$R" -- -d $C --set -c voltage_target=5 -c current_limit=9
    emu_case "refused then set" --keep-header --stderr --expect-fail --filter "$R" -- -d $C --set -c current_limit=9 -c voltage_target=5

    # The dummy start.
    emu_case "capture samples" --keep-header --stderr --expect-fail -- -d $C --samples 1
    emu_case "capture set" --keep-header --stderr --expect-fail -- -d $C -c voltage_target=5 --samples 1
    emu_case "capture continuous" --keep-header --stderr --expect-fail -- -d $C --continuous
    emu_case "capture time" --keep-header --stderr --expect-fail -- -d $C --time 10
    # With no limit to set, the session refuses: no channels (session.c:790-793).
    emu_case "lib capture" --libsigrok --keep-header --stderr --expect-fail -- -d $C

    # The write failures. C's words and oxrok's differ, as oxrok says which it was,
    # and C fails too, so these compare the failure and the wire, and the "says so"
    # cases hold oxrok's words. The unplug falls in the pause after the first command,
    # and the writes that follow all fail the same way, so how soon the hangup lands
    # does not change how this ends. The .tx stops at the unplug for the same reason.
    emu_case "unplug between settings" --unplug out=2:1 --keep-header --no-output --expect-fail --tx-filter '/^unplug$/q' -- -d $C --set -c voltage_target=5 -c voltage_target=6 -c voltage_target=7 -c voltage_target=8 -c voltage_target=9
    # Whether the wait for V050 or the write of V060 meets the hangup is that race, and
    # their causes differ: the fold is of those two only, and the unit tests pin the rest.
    emu_case "unplug says so" --no-oracle-only --unplug out=2:1 --keep-header --stderr --expect-fail --filter "s/the port failed sending V0[56]0: .*/the port failed sending V0[56]0/" --tx-filter '/^unplug$/q' -- -d $C --set -c voltage_target=5 -c voltage_target=6 -c voltage_target=7 -c voltage_target=8 -c voltage_target=9
    emu_case "refusal says so" --no-oracle-only --keep-header --stderr --expect-fail -- -d $C --set -c voltage_target=36
    # The supply stops reading. C's tty holds 16 writes, and the 17th has its 15 ms and
    # fails. oxrok's second set fails, as its frame doesn't leave the queue.
    emu_options out-hold=1
    emu_tty_closing_wait 100
    S=""
    for v in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
        S="$S -c voltage_target=$v"
    done
    emu_case "stops reading" --keep-header --no-output --expect-fail --timeout 20 -- -d $C --set $S
    # With fewer than fill the queue, C says nothing went wrong.
    emu_case "stays in the queue" --oracle-differs "$QUEUE_BUG" --keep-header --stderr --expect-fail --filter "$L" --timeout 20 -- -d $C --set -c voltage_target=5 -c voltage_target=6
    emu_tty_closing_wait 3000
    emu_options
}

emu_conns() {
    C=conrad-digi-35-cpu:conn=$EMU_TTY
}
