# A serial multimeter on the CDC-ACM shell (emu/src/models/serial-dmm.rs),
# the shell's pilot, on both tools. Sourced by tools/diffcli.sh --emu;
# see docs/USB.md, "Writing a serial model".
EMU_MODEL=serial-dmm
EMU_CDC=1
EMU_DRIVER="metex-m3640d uni-t-ut61e-ser uni-t-ut60a-ser"
EMU_FULL=$EMU_DRIVER
EMU_GCOV=serial-dmm
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c src/dmm/metex14.c"

emu_cases() {
    PARITY_BUG="libsigrok swaps odd and even: serial.c hands serial_set_params libserialport's SP_PARITY_EVEN (2) for e, which serial_libsp.c takes for odd"
    HUP_BUG="serial-dmm receive_data takes a hung up port for a timeout, and a meter it does not ask writes nothing that could fail (protocol.c:254-262)"
    BLOCK_BUG="serial_write_blocking without a timeout waits for ever in sp_blocking_write once the kernel's write buffers are full"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"
    M=metex-m3640d:conn=$EMU_TTY
    E=uni-t-ut61e-ser:conn=$EMU_TTY
    F=uni-t-ut60a-ser:conn=$EMU_TTY

    # A meter that is asked for each packet.
    emu_case "metex scan" --keep-header -- -d $M --scan
    emu_case "metex show" --keep-header -- -d $M --show
    emu_case "metex samples 1" -- -d $M --samples 1
    emu_case "metex samples 3" -- -d $M --samples 3
    emu_case "metex serialcomm as the table" -- -d $M:serialcomm=1200/7n2/rts=0/dtr=1 --samples 1
    # No such node, and a line the meter is not on: no device, no error.
    emu_case "metex no such node" --keep-header --no-output -- -d metex-m3640d:conn=/dev/ttyACM97 --scan
    emu_case "metex wrong baud" --keep-header --no-output -- -d $M:serialcomm=9600/7n2/rts=0/dtr=1 --scan
    emu_case "metex wrong frame" --keep-header --no-output -- -d $M:serialcomm=1200/8n1/rts=0/dtr=1 --scan

    # The meter misbehaving.
    serial_dmm_options answers=0
    emu_case "metex never answers" --keep-header --no-output -- -d $M --scan
    serial_dmm_options answers=1
    # The scan's request is answered, the acquisition's is not; it asks
    # again at each read that times out, as often as the time lets it.
    emu_case "metex stops answering" --no-output \
        --tx-filter 's/^bulk_out 02 (44)+ -> [0-9]+$/bulk_out 02 44... -> N/' -- -d $M --time 500ms
    serial_dmm_options noise=3f3f3f
    emu_case "metex noise before each packet" -- -d $M --samples 2
    serial_dmm_options noise=$(printf '3f%.0s' $(seq 1 40))
    emu_case "metex much noise before the packet" -- -d $M --samples 1
    serial_dmm_options packet=$(printf 'DC  1.234 V  X' | od -An -tx1 | tr -d ' \n')
    emu_case "metex invalid packet" --keep-header --no-output -- -d $M --scan
    serial_dmm_options packet=$(printf 'DC  1.234 V  \rAC  0.500 V  \r' | od -An -tx1 | tr -d ' \n' | sed -E 's/(.{28})/\1,/')
    emu_case "metex two readings in turn" -- -d $M --samples 3

    # The USB side misbehaving, the shell's options.
    serial_dmm_options chunk=1
    emu_case "metex one byte per transfer" -- -d $M --samples 2
    serial_dmm_options stall=line
    emu_case "metex line coding stalled" --keep-header --no-output -- -d $M --scan
    serial_dmm_options stall=lines
    # sigrok-cli stops at the failed RTS and never sets the line, oxrok
    # has set it (serialport's open comes before RTS can): no device.
    emu_case "metex control lines stalled" --keep-header --no-output \
        --tx-filter '/^ctl_out 21 20 /d' -- -d $M --scan
    # The device stops reading. sigrok-cli asks every 50 ms until its 16th
    # write blocks for good, which no limit interrupts. oxrok ends at the
    # time limit, and the kernel holds its close for closing_wait, 30 s.
    serial_dmm_options out-hold=1
    emu_case "metex stops reading" --oracle-bug "$BLOCK_BUG" --oracle-hangs --no-output --timeout 40 \
        --tx-filter '/^bulk_out 02 /d' -- -d $M --time 200ms
    serial_dmm_options

    # A meter that sends on its own, the ESP32's EMIT cases of --hw.
    serial_dmm_options meter=es51922
    emu_case "ut61e scan" --driver uni-t-ut61e-ser --oracle-differs "$PARITY_BUG" --keep-header -- -d $E --scan
    emu_case "ut61e samples 1" --driver uni-t-ut61e-ser --oracle-differs "$PARITY_BUG" -- -d $E --samples 1
    emu_case "ut61e samples 3" --driver uni-t-ut61e-ser --oracle-differs "$PARITY_BUG" -- -d $E --samples 3
    serial_dmm_options meter=fs9721
    emu_case "ut60a samples 2" --driver uni-t-ut60a-ser -- -d $F --samples 2
    serial_dmm_options meter=fs9721 burst=0
    emu_case "ut60a never sends" --driver uni-t-ut60a-ser --keep-header --no-output -- -d $F --scan
    # The kernel clears the halt and goes on: no tool sees it.
    serial_dmm_options meter=fs9721 in-stall=1
    emu_case "ut60a a stalled read" --driver uni-t-ut60a-ser -- -d $F --samples 2
    serial_dmm_options

    # An unplug with the tty open, and an interrupt.
    emu_case "metex unplug" --unplug in=2:3 --timeout 15 -- -d $M --samples 5
    serial_dmm_options meter=fs9721
    # The scan takes two packets (in 2:1 and 2:2) to find a valid one.
    emu_case "ut60a unplug" --driver uni-t-ut60a-ser --oracle-bug "$HUP_BUG" --oracle-hangs --unplug in=2:4 --timeout 15 -- -d $F --samples 5
    serial_dmm_options answers=3
    # The driver asks again every 50 ms until the signal lands, so what
    # follows the mark depends on the pace.
    emu_case "metex interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 \
        --tx-filter '/^interrupt$/q' -- -d $M --continuous
    serial_dmm_options meter=fs9721 burst=3
    emu_case "ut60a interrupt" --driver uni-t-ut60a-ser --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 15 -- -d $F --continuous
    serial_dmm_options
}

# The model's options for the cases that follow (see
# tools/emu/ftdi-la.sh); none restores EMU_OPTIONS.
serial_dmm_options() {
    set -- options "$@"
    emu_cmd "$*" >/dev/null || emu_fail "serial-dmm: the emulator refused '$*'"
}
