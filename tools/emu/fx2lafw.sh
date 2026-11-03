# The fx2lafw logic analyzer (emu/src/models/fx2lafw.rs): an FX2
# without firmware that takes the upload and renumerates, as the bare
# Cypress board (16 channels), a Saleae Logic (8 channels) and sigrok's
# fx2grok-flat (1d50:608c, 8 channels), and the same boards with the
# firmware already running. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a device model".
EMU_MODEL=fx2lafw
EMU_OPTIONS="-o profile=cypress-fx2"
EMU_DRIVER=fx2lafw
EMU_GCOV_SHARED="src/usb.c src/ezusb.c"
# oxrok has the driver: its cases must run full.
EMU_FULL=fx2lafw

# The model's options for the cases that follow: every case's reset
# builds the model with them, and re-attaches the device when its
# descriptors change.
fx2lafw_options() {
    # The emulator trims what it acknowledges: no trailing space.
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "fx2lafw: the emulator refused '$*'"
}

emu_cases() {
    F=fx2lafw
    D8=D0,D1,D2,D3,D4,D5,D6,D7
    # A device that renumerated has an address the READY line did not
    # report, and a different one for each tool.
    C='s/conn(=|: )[0-9]+\.[0-9]+/conn\1BUS.ADDR/g'

    # The bare Cypress FX2: every run uploads the firmware first, in the
    # scan, and the device renumerates before it is opened.
    fx2lafw_options profile=cypress-fx2
    emu_case "fx2 scan" --keep-header -- -d $F --scan
    emu_case "fx2 show" --keep-header --filter "$C" -- -d $F --show
    emu_case "fx2 by ids" -- -d $F:conn=04b4.8613 --samples 8 -O hex
    emu_case "fx2 8ch 20k" -- -d $F -C $D8 -c samplerate=20k --samples 600 -O hex
    emu_case "fx2 8ch 1M" -- -d $F -C $D8 -c samplerate=1M --samples 64 -O bits
    emu_case "fx2 8ch 3M" --filter '1,/^D7/!d' -- -d $F -C $D8 -c samplerate=3M --samples 40000 -O hex
    emu_case "fx2 8ch 24M" -- -d $F -C $D8 -c samplerate=24M --samples 1024 -O hex
    emu_case "fx2 8ch 48M" -- -d $F -C $D8 -c samplerate=48M --samples 100 -O hex
    emu_case "fx2 16ch 50k" -- -d $F -c samplerate=50k --samples 300 -O hex
    emu_case "fx2 16ch 12M" --filter '1,/^D15/!d' -- -d $F -c samplerate=12M --samples 2000 -O bits
    emu_case "fx2 one sample" -- -d $F -C $D8 -c samplerate=1M --samples 1 -O bits
    # Triggers on the 16 channel board with all channels: with fewer,
    # libsigrok's soft trigger takes two bytes per sample (all logic
    # channels) while the stream has one, see the model's comments.
    emu_case "fx2 trigger 16ch" -- -d $F -c samplerate=1M --samples 24 -t D9=r -O bits
    emu_case "fx2 trigger 16ch ratio" -- -d $F -c samplerate=500k:captureratio=50 --samples 40 -t D3=f -O bits
    emu_case "fx2 frames" -- -d $F -C $D8 -c samplerate=1M:limit_frames=2 --samples 8 -O bits
    emu_case "fx2 probe names" --keep-header -- -d $F:probe_names=CLK,DATA,CS --scan
    emu_case "fx2 bad rate" --expect-fail --stderr --filter '1!d; s/.*/refused/' -- -d $F -c samplerate=7M --samples 8 -O hex
    # Several transfers of 512 bytes (10 ms at 20 kHz): a trigger in the
    # third, a time limit (sigrok-cli makes it a sample limit).
    emu_case "fx2 16ch 20k trigger late" -- -d $F -c samplerate=20k --samples 300 -t D9=r -O hex
    emu_case "fx2 time limit" -- -d $F -c samplerate=50k --time 30 -O hex
    emu_case "fx2 16ch at 24M" --expect-fail --stderr --filter '/Unable to sample|maximum sample rate/!d; s/.*/refused/' -- -d $F -c samplerate=24M --samples 16 -O hex

    # A Saleae Logic, 8 channels.
    fx2lafw_options profile=saleae-logic
    emu_case "logic scan" --keep-header -- -d $F --scan
    emu_case "logic 25k" -- -d $F -c samplerate=25k --samples 100 -O hex
    emu_case "logic 24M" --filter '1,/^D7/!d' -- -d $F -c samplerate=24M --samples 3000 -O hex
    emu_case "logic trigger rising" -- -d $F -c samplerate=1M --samples 32 -t D5=r -O bits
    emu_case "logic trigger falling" -- -d $F -c samplerate=4M --samples 20 -t D4=f -O bits
    emu_case "logic trigger ratio" -- -d $F -c samplerate=1M:captureratio=25 --samples 32 -t D6=1 -O bits
    emu_case "logic trigger two matches" -- -d $F -c samplerate=200k --samples 16 -t D0=0,D7=f -O bits
    emu_case "logic trigger frames" -- -d $F -c samplerate=1M:limit_frames=3 --samples 6 -t D2=r -O bits
    emu_case "logic 20k six transfers" -- -d $F -c samplerate=20k --samples 3000 -O hex
    emu_case "logic frames across transfers" -- -d $F -c samplerate=20k:limit_frames=3 --samples 300 -O hex
    emu_case "logic trigger frames across transfers" -- -d $F -c samplerate=20k:limit_frames=3 --samples 200 -t D7=f -O hex

    # sigrok's fx2grok-flat: the same ids before and after the upload.
    fx2lafw_options profile=sigrok-fx2-8ch
    emu_case "fx2grok scan" --keep-header -- -d $F --scan
    emu_case "fx2grok 2M" -- -d $F -c samplerate=2M --samples 50 -O hex

    # Already flashed: the firmware runs from the start, no upload.
    fx2lafw_options profile=cypress-fx2 flashed=1
    emu_case "flashed fx2 scan" --keep-header --filter "$C" -- -d $F --scan
    emu_case "flashed fx2 show" --keep-header --filter "$C" -- -d $F --show
    emu_case "flashed fx2 8ch 2M" -- -d $F -C $D8 -c samplerate=2M --samples 128 -O hex
    emu_case "flashed fx2 16ch 6M" -- -d $F -c samplerate=6M --samples 100 -O hex
    fx2lafw_options profile=saleae-logic flashed=1
    emu_case "flashed logic scan" --keep-header --filter "$C" -- -d $F --scan
    emu_case "flashed logic 16M" -- -d $F -c samplerate=16M --samples 50 -t D2=r -O bits
    # A frame that ends where a transfer ends: libsigrok's
    # receive_transfer() then hands its NULL soft trigger to
    # soft_trigger_logic_check() (protocol.c:484) and crashes. On the
    # flashed board, as the stderr golden must not depend on whether
    # libusb opened the renumerated device before its node was
    # accessible (the C driver logs that and tries again).
    emu_case "logic frame on a transfer boundary" --oracle-bug "receive_transfer() crashes when a frame without trigger ends at the end of a transfer" -- -d $F -c samplerate=20k:limit_frames=2 --samples 512 -O hex
    fx2lafw_options profile=sigrok-fx2-8ch flashed=1
    emu_case "flashed fx2grok by ids" --filter "$C" -- -d $F:conn=1d50.608c --samples 8 -O hex
    fx2lafw_options
}
