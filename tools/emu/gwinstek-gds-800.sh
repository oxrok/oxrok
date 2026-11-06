# A GW Instek GDS-800 scope (emu/src/models/gwinstek-gds-800.rs), on its serial port,
# on the CDC-ACM shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a
# SCPI model".
EMU_MODEL=gwinstek-gds-800
EMU_CDC=1
EMU_DRIVER=gwinstek-gds-800
EMU_GCOV=gwinstek-gds-800
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    # A capture's output in few lines, and through --digest the hash of all of it.
    A='1~64!d'
    G=gwinstek-gds-800:conn=$EMU_TTY

    emu_case "scan" --keep-header -- -d $G --scan
    emu_case "show" --keep-header -- -d $G --show
    emu_case "get samplerate" -- -d $G --get samplerate
    emu_case "get limit_frames" -- -d $G --get limit_frames
    emu_case "set limit_frames" --no-output -- -d $G --config limit_frames=3 --set
    emu_case "serialcomm" --keep-header -- -d $G:serialcomm=38400/8n1 --scan

    emu_case "frame" --digest --filter "$A" -- -d $G --frames 1 -O analog
    emu_case "frame of ch1" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_case "frame of ch2" --digest --filter "$A" -- -d $G -C CH2 --frames 1 -O analog
    emu_case "two frames" --digest --filter "$A" -- -d $G --frames 2 -O analog
    emu_case "two frames of ch2" --digest --filter "$A" -- -d $G -C CH2 --frames 2 -O analog
    emu_case "frames by config" --digest --filter "$A" -- -d $G -C CH1 -c limit_frames=2 --frames 2 -O analog
    emu_case "frame keeps the header" --keep-header --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog

    # What sigrok-cli asks of a driver with frames alone.
    emu_case "samples is no limit here" --expect-fail --stderr -- -d $G --samples 10 -O analog
    emu_case "time is no limit here" --expect-fail --stderr -- -d $G --time 100 -O analog

    # Who it is.
    emu_options model=GDS-840C firmware=V.2.10
    emu_case "scan gds-840c" --keep-header -- -d $G --scan
    emu_options idn=GW,GDS-2204,P1,V.1
    emu_case "scan another gw scope" --no-output -- -d $G --scan
    emu_options idn=Rigol,GDS-820,P1,V.1
    emu_case "scan another maker" --no-output -- -d $G --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $G --scan
    emu_options idn=GW,GDS-820,V.2.03
    emu_case "scan with three fields" --keep-header -- -d $G --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $G --scan
    emu_options silent-at=1
    emu_case "scan once silent" --no-output -- -d $G --scan
    emu_options late-at=1:5
    emu_case "scan late" --keep-header -- -d $G --scan
    emu_options

    # Record lengths: each digit count, and one past what a tty buffer holds.
    emu_options points=1250
    emu_case "1250 points" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options points=5000
    emu_case "5000 points" --digest --filter "$A" -- -d $G -C CH2 --frames 1 -O analog
    emu_options points=50000
    emu_case "50000 points" --digest --filter '1~4096!d' -- -d $G -C CH1 --frames 1 -O analog
    emu_options

    # What it says of its scale.
    emu_options scale1=100mV scale2=2V
    emu_case "scale in mv and v" --digest --filter "$A" -- -d $G --frames 1 -O analog
    emu_options scale1=2.000E-03 scale2=5.000E+00
    emu_case "scale in nr3" --digest --filter "$A" -- -d $G --frames 1 -O analog
    emu_options

    # Pieces, and a pause before a block.
    emu_options chunk=32
    emu_case "block in pieces" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options late-at=2:10
    emu_case "block a while late" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options late-at=3:5
    emu_case "scale a while late" --digest --filter "$A" -- -d $G -C CH1 --frames 1 -O analog
    emu_options
}
