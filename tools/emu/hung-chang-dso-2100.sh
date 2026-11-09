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
# therefore compare the transactions alone (--filter d --no-output).
#
# A readout is 1003 nStrobe pulses, each a DCR write, a data read and a
# DCR write: some 3000 transactions a 1000 sample subframe. The cases
# that sweep a setting drop the readout's lines from the transactions
# (HC_NO_READOUT; the DCR then is 0x3a/0x3b for A, 0x38/0x39 for B, the
# reads show it), so each setting's mailbox bytes are compared in few
# lines. The cases before them keep everything.
EMU_MODEL=hung-chang-dso-2100
EMU_DRIVER=hung-chang-dso-2100
EMU_PARPORT=1
# uss720's claim ends with a register read it does not wait for, whose late
# answer can undo libieee1284's PPDATADIR right after it: about one run in
# eight lost the direction bit (DCR 0x12 for 0x32) and read the latch. The
# oracle waits 20 ms after each claim, as oxrok does (CLAIM_SETTLE).
EMU_ORACLE_PPCLAIM_SETTLE_MS=20

HC_NO_READOUT='/^ctl_out 40 04 023[89ab] /d; /^ctl_in 40 03 0000 0000 7 -> 003[89ab]/d'
# A scope that stops answering: C's mailbox reads go on until a timeout,
# so how many there are is the pace's. Runs of a repeated line count once.
HC_UNIQ='$!N; /^(.*)\n\1$/!P; D'

emu_cases() {
    P=hung-chang-dso-2100:conn=$EMU_PARPORT_NAME
    emu_case "dso2100 scan" --keep-header -- -d $P --scan
    emu_case "dso2100 show" --keep-header -- -d $P --show
    emu_case "dso2100 get samplerate" -- -d $P --get samplerate
    emu_case "dso2100 get vdiv B" -- -d $P -g B --get vdiv
    emu_case "dso2100 no conn" --keep-header --no-output -- -d hung-chang-dso-2100 --scan
    emu_case "dso2100 no such port" --stderr -- -d hung-chang-dso-2100:conn=parport9 --scan

    emu_case "dso2100 A one frame" --filter d --no-output -- -d $P -C A --frames 1 -O analog
    emu_case "dso2100 B one frame" --filter d --no-output -- -d $P -C B --frames 1 -O analog
    emu_case "dso2100 continuous" --stderr --expect-fail -- -d $P -C A --continuous -O analog
    emu_case "dso2100 csv" --stderr --expect-fail -- -d $P -C A --frames 1 -O csv
    emu_case "dso2100 no channel" --stderr --expect-fail -- -d $P --frames 1 -O analog
    emu_case "dso2100 both channels" --stderr --expect-fail -- -d $P -C A,B --frames 1 -O analog
    emu_case "dso2100 two frames" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 2 -O analog
    emu_case "dso2100 buffer 1500" --filter d --no-output -- -d $P -C A -c buffersize=1500 --frames 1 -O analog
    emu_case "dso2100 buffer 5998" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C B -c buffersize=5998 --frames 1 -O analog
    emu_case "dso2100 buffer 10238" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c buffersize=10238 --frames 1 -O analog

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
    emu_case "dso2100 probe 0" --stderr --expect-fail -- -d $P -C A -g A -c probe_factor=0 --frames 1 -O analog
    emu_case "dso2100 trigger B rising" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c triggersource=B:triggerslope=r --frames 1 -O analog
    emu_case "dso2100 trigger EXT" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A -c triggersource=EXT --frames 1 -O analog
    emu_case "dso2100 bad samplerate" --stderr --expect-fail -- -d $P -C A -c samplerate=3M --frames 1 -O analog

    # The scope's own pace and faults, through the model's options.
    hc_options trigger=12
    emu_case "dso2100 late trigger" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    hc_options busy=1
    emu_case "dso2100 busy" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    hc_options scope=absent
    emu_case "dso2100 no scope" --keep-header --no-output -- -d $P --scan
    hc_options bad-sig=1
    emu_case "dso2100 bad signature once" --filter d --no-output --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    hc_options bad-sig=4
    emu_case "dso2100 bad signature 4 times" --stderr --tx-filter "$HC_NO_READOUT" -- -d $P -C A --frames 1 -O analog
    # The 19th mailbox write is the arming, after config_commit's 18.
    hc_options odd-state=19
    emu_case "dso2100 odd state at arming" --stderr --expect-fail -- -d $P -C A --frames 1 -O analog
    # Silent from the 21st write: the second 0x99 of the trigger poll.
    hc_options silent-after=20
    emu_case "dso2100 silent while armed" --hangs --timeout 10 --no-output --tx-filter "$HC_UNIQ" -- -d $P -C A --frames 1 -O analog
    # Silent from the 23rd: the 4 after the readout, on the way back to idle.
    hc_options silent-after=22
    emu_case "dso2100 silent after the readout" --filter d --no-output --tx-filter "$HC_NO_READOUT; $HC_UNIQ" -- -d $P -C A --frames 1 -O analog
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
