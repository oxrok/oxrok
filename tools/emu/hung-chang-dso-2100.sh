# The Hung-Chang DSO-2100 behind a USS720 USB to parallel cable
# (emu/src/models/hung-chang-dso-2100.rs). Sourced by tools/diffcli.sh
# --emu; see docs/USB.md, "Writing a device model".
#
# The kernel's uss720 makes the cable a parallel port and ppdev makes it
# /dev/parportN (EMU_PARPORT=1, see the head of tools/diffcli.sh): both
# tools reach the scope through the kernel, libsigrok through
# libieee1284, so the transactions are the USS720 register requests
# their ppdev ioctls cause.
#
# libsigrok's driver hands its samples on with a count of 4294967295
# (protocol.c push_samples: `while (num--)` runs num down before
# `analog.num_samples = num`, since 99f8fa25 of 2016), so sigrok-cli
# prints no sample: the analog output's g_try_malloc of 16 GiB fails and
# it writes nothing, at exit 0; csv's g_malloc aborts. The capture cases
# therefore compare the transactions alone (--filter d --no-output), and
# the "samples" cases pin oxrok's samples against its own goldens
# (--oracle-differs, every 50th line and the digest of the whole).
#
# A readout is 1003 nStrobe pulses, each a DCR write, a data read and a
# DCR write: some 3000 transactions a 1000 sample subframe. The cases
# that sweep a setting drop the readout's lines from the transactions
# (HC_NO_READOUT; the DCR then is 0x3a/0x3b for A, 0x38/0x39 for B, the
# reads show it), so each setting's mailbox bytes are compared in few
# lines. The cases before them keep everything.
EMU_MODEL=hung-chang-dso-2100
EMU_DRIVER=hung-chang-dso-2100
# oxrok has the driver: its cases must run full.
EMU_FULL=hung-chang-dso-2100
EMU_PARPORT=1
# uss720 is the cable's driver for both tools: emu_unbind leaves it bound.
EMU_KEEP_BOUND=uss720
# uss720's claim ends with a register read it does not wait for, whose late
# answer can undo libieee1284's PPDATADIR right after it: about one run in
# eight lost the direction bit (DCR 0x12 for 0x32) and read the latch. The
# oracle waits 20 ms after each claim, as oxrok does (CLAIM_SETTLE).
EMU_ORACLE_PPCLAIM_SETTLE_MS=20

HC_NO_READOUT='/^ctl_out 40 04 023[89ab] /d; /^ctl_in 40 03 0000 0000 7 -> 003[89ab]/d'
# A scope that stops answering: C's mailbox reads go on until a timeout,
# so how many there are is the pace's. Runs of a repeated line count once.
HC_UNIQ='$!N; /^(.*)\n\1$/!P; D'
HC_THIN='1~50!d'
# Every 50th line, and where the plain steps' 5000 samples meet the
# interleaved ones, and the frame's end: a sample put in the wrong place
# shows as a step in the wave there.
HC_WIN='1~50b; 4991,5011b; 10231,10238b; d'
HC_COUNT="libsigrok's push_samples sends num_samples 4294967295 (protocol.c, since 99f8fa25)"
# oxrok's log prefix made sigrok-cli's, and the detail of the reason oxrok's
# message adds to sigrok-cli's own; the "words" cases keep oxrok's own.
HC_O='oxrok: [a-z]+: oxrok::drivers::hung_chang_dso_2100(::protocol)?:'
HC_W="s/^$HC_O /sr: hung-chang-dso-2100: /; s/^(Failed to set device option '[a-z_]+': invalid argument): .*\$/\\1./; s/^(Failed to set channels): .*\$/\\1./"
# sigrok-cli's two lines rewritten as oxrok's one, as tools/emu/ipdbg-la.sh.
HC_NOCH='s/^sr: session: hung-chang-dso-2100 device \(null\) has no enabled channels\.$/hung-chang-dso-2100 device has no enabled channels/
/^Failed to start session\.$/d'

emu_cases() {
    P=hung-chang-dso-2100:conn=$EMU_PARPORT_NAME
    emu_case "dso2100 scan" --keep-header -- -d $P --scan
    emu_case "dso2100 show" --keep-header -- -d $P --show
    emu_case "dso2100 get samplerate" -- -d $P --get samplerate
    emu_case "dso2100 get vdiv B" -- -d $P -g B --get vdiv
    emu_case "dso2100 no conn" --keep-header --no-output -- -d hung-chang-dso-2100 --scan
    emu_case "dso2100 no such port" --stderr --filter "$HC_W" -- -d hung-chang-dso-2100:conn=parport9 --scan

    emu_case "dso2100 A one frame" --filter d --no-output -- -d $P -C A --frames 1 -O analog
    emu_case "dso2100 B one frame" --filter d --no-output -- -d $P -C B --frames 1 -O analog
    emu_case "dso2100 continuous" --stderr --expect-fail -- -d $P -C A --continuous -O analog
    emu_case "dso2100 csv" --oracle-bug "$HC_COUNT; csv aborts on a 16 GiB g_malloc" --digest --filter "$HC_THIN" -- -d $P -C A --frames 1 -O csv:header=false
    # sigrok-cli opens the scope, refuses, and leaves without its close;
    # oxrok's close then resets the port (its last two lines, 0x30 and 0x10).
    emu_case "dso2100 no channel" --stderr --expect-fail --filter "$HC_NOCH" --tx-filter '34,35d' -- -d $P --frames 1 -O analog
    emu_case "dso2100 both channels" --stderr --expect-fail --filter "$HC_W" -- -d $P -C A,B --frames 1 -O analog
    emu_case "dso2100 two frames" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 2 -O analog
    emu_case "dso2100 buffer 1500" --filter d --no-output -- -d $P -C A -c buffersize=1500 --frames 1 -O analog
    # The interleaved readouts kept whole, on B: which data line each ADC's
    # pass reads, and the strobe counts, against C's.
    emu_case "dso2100 buffer 5998" --filter d --no-output -- -d $P -C B -c buffersize=5998 --frames 1 -O analog
    emu_case "dso2100 buffer 10238" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c buffersize=10238 --frames 1 -O analog

    # The samples, which sigrok-cli never prints (see the head): oxrok's,
    # pinned to its own goldens.
    S='--digest --filter'
    emu_case "dso2100 A samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" -- -d $P -C A --frames 1 -O analog
    emu_case "dso2100 B samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C B --frames 1 -O analog
    emu_case "dso2100 two frames samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 2 -O analog
    emu_case "dso2100 buffer 1500 samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c buffersize=1500 --frames 1 -O analog
    emu_case "dso2100 buffer 5998 samples" --oracle-differs "$HC_COUNT" $S "$HC_WIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C B -c buffersize=5998 --frames 1 -O analog
    emu_case "dso2100 buffer 10238 samples" --oracle-differs "$HC_COUNT" $S "$HC_WIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c buffersize=10238 --frames 1 -O analog
    emu_case "dso2100 vdiv 10mV samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -g A -c vdiv=10mV --frames 1 -O analog
    emu_case "dso2100 vdiv 5V probe 1 samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -g A -c vdiv=5V:probe_factor=1 --frames 1 -O analog
    emu_case "dso2100 coupling AC samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -g A -c coupling=AC --frames 1 -O analog
    emu_case "dso2100 coupling GND samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C B -g B -c coupling=GND --frames 1 -O analog
    emu_case "dso2100 rate 1k samples" --oracle-differs "$HC_COUNT" $S "$HC_THIN" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c samplerate=1k --frames 1 -O analog
    # oxrok's own words where the cases above map them to sigrok-cli's.
    emu_case "dso2100 both channels, oxrok's words" --no-oracle-only --stderr --expect-fail -- -d $P -C A,B --frames 1 -O analog
    emu_case "dso2100 no such port, oxrok's words" --no-oracle-only --stderr -- -d hung-chang-dso-2100:conn=parport9 --scan

    # Every timebase and every volts/div, and the rest of the settings.
    for r in 100M 50M 25M 20M 10M 5M 2500k 2M 1M 500k 250k 200k 100k 50k 25k 20k \
        10k 5k 2500 2k 1k 500 250 200 100 50 25 20; do
        emu_case "dso2100 rate $r" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c samplerate=$r --frames 1 -O analog
    done
    for v in 10mV 20mV 50mV 100mV 200mV 500mV 1V 2V 5V; do
        emu_case "dso2100 vdiv $v" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -g A -c vdiv=$v --frames 1 -O analog
    done
    emu_case "dso2100 vdiv B 200mV" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C B -g B -c vdiv=200mV --frames 1 -O analog
    emu_case "dso2100 coupling AC" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -g A -c coupling=AC --frames 1 -O analog
    emu_case "dso2100 coupling GND" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C B -g B -c coupling=GND --frames 1 -O analog
    emu_case "dso2100 probe 1" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -g A -c probe_factor=1 --frames 1 -O analog
    emu_case "dso2100 probe 0" --stderr --expect-fail --filter "$HC_W" -- -d $P -C A -g A -c probe_factor=0 --frames 1 -O analog
    emu_case "dso2100 trigger B rising" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c triggersource=B:triggerslope=r --frames 1 -O analog
    emu_case "dso2100 trigger EXT" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c triggersource=EXT --frames 1 -O analog
    emu_case "dso2100 bad samplerate" --stderr --expect-fail --filter "$HC_W" -- -d $P -C A -c samplerate=3M --frames 1 -O analog

    # The scope's own pace and faults, through the model's options.
    hc_options trigger=12
    emu_case "dso2100 late trigger" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    # The scope filling 10240 samples after its trigger at 2 kHz, 5.12 s,
    # with its mailbox reading 0 meanwhile: 640 polls of nothing, then the
    # frame. Each poll's reads count once (their number is the pace's).
    hc_options fill=rate
    emu_case "dso2100 fill at 2 kHz" --filter d --no-output --tx-filter "$HC_NO_READOUT; $HC_UNIQ" -- -d $P -C A -c samplerate=2k --frames 1 -O analog
    hc_options busy=1
    emu_case "dso2100 busy" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    hc_options scope=absent
    emu_case "dso2100 no scope" --keep-header --no-output -- -d $P --scan
    hc_options bad-sig=1
    emu_case "dso2100 bad signature once" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    hc_options bad-sig=4
    emu_case "dso2100 bad signature 4 times" --stderr --filter "$HC_W" --oxrok-fails "Failed to read frame without transfer errors" --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    # The 19th mailbox write is the arming, after config_commit's 18.
    hc_options odd-state=19
    emu_case "dso2100 odd state at arming" --oracle-bug "libsigrok stops the device whose start failed: Cannot remove non-existing event source" --stderr --expect-fail -- -d $P -C A --frames 1 -O analog
    # The 21st write, the second 0x99, answered with a state the poll does
    # not know.
    hc_options odd-state=21
    emu_case "dso2100 odd state while armed" --stderr --filter "$HC_W" --oxrok-fails "Unexpected state 0x42 while checking for trigger" -- -d $P -C A --frames 1 -O analog
    # With buffersize 1500: the 40th write, the 3 that extracts the second
    # readout, answered with an unknown state; and silence from the 23rd,
    # the 4 after the first readout, on the way to the second.
    hc_options odd-state=40
    emu_case "dso2100 odd state at the second extraction" --no-output --filter d --oxrok-fails "answered 0x42 to the extraction of a readout" --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c buffersize=1500 --frames 1 -O analog
    hc_options silent-after=22
    emu_case "dso2100 silent between readouts" --no-output --filter d --oxrok-fails "stopped answering on the way to state 0x01" --tx-filter "$HC_NO_READOUT; $HC_UNIQ" -- -d $P -C A -c buffersize=1500 --frames 1 -O analog
    # Silent from the 21st write: the second 0x99 of the trigger poll.
    hc_options silent-after=20
    emu_case "dso2100 silent while armed" --oracle-bug "C polls a silent mailbox for ever" --oracle-hangs --timeout 15 --oxrok-fails "No answer from the DSO-2100 in 600 trigger polls" --no-output --tx-filter "$HC_UNIQ" -- -d $P -C A --frames 1 -O analog
    # Silent from the 23rd: the 4 after the readout, on the way back to idle.
    hc_options silent-after=22
    emu_case "dso2100 silent after the readout" --filter d --no-output --tx-filter "$HC_NO_READOUT; $HC_UNIQ" -- -d $P -C A --frames 1 -O analog
    # No interrupt case: oxrok's stop comes after however many trigger polls
    # the signal's pace allows, each 13 transactions, and sigrok-cli dies at
    # SIGINT. The stop while armed is in the driver's unit tests.
    # No case pulls the cable: on 2026-11-09 the session that first did
    # (the unplug in a readout, sigrok-cli holding the port claimed) took
    # usbtester3's kernel down with it, with nothing in the journal; it
    # came back by itself a minute later. The port's going away is in the
    # driver's unit tests only.
    hc_options
}

# The model's options for the cases that follow (see
# tools/emu/ftdi-la.sh); none restores EMU_OPTIONS.
hc_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "hung-chang-dso-2100: the emulator refused '$*'"
}
