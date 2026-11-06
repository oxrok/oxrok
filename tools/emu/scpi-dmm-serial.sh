# A SCPI meter on a serial port (emu/src/models/scpi-dmm-serial.rs), on the CDC-ACM
# shell, as an OWON or a GW Instek is reached. Sourced by tools/diffcli.sh --emu; see
# docs/USB.md, "Writing a SCPI model". The driver's own paths are scpi-dmm.sh's: this
# holds what a tty changes.
EMU_MODEL=scpi-dmm-serial
EMU_CDC=1
EMU_DRIVER=scpi-dmm
EMU_GCOV=scpi-dmm
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_serial.c src/serial.c src/serial_libsp.c"

emu_cases() {
    D=scpi-dmm:conn=$EMU_TTY
    # An XDM2041 with no *OPC?: the probe waits it out, and it is never asked again.
    emu_case "scan" --keep-header -- -d $D --scan
    emu_case "show" --keep-header -- -d $D --show
    emu_case "capture" -- -d $D --samples 2 -O analog
    emu_case "serialcomm" --keep-header -- -d $D:serialcomm=115200/8n1 --scan
    emu_case "serialcomm in a capture" -- -d $D:serialcomm=9600/8n1 --samples 1 -O analog
    emu_case "set a function" --no-output -- -d $D -c measured_quantity=voltage/ac --set
    emu_options opc=1
    emu_case "scan with opc" --keep-header -- -d $D --scan
    emu_case "capture with opc" -- -d $D --samples 2 -O analog
    emu_options model=GDM8251A func=RES readings=1.20000E+08
    emu_case "a gw" -- -d $D --samples 1 -O analog
    emu_options model=GDM8255A eol=crlf
    emu_case "a gw with cr lf" -- -d $D --samples 1 -O analog
    # What a line can do: pieces, a reading slower than a read, one that doesn't come.
    emu_options opc=1 chunk=3
    emu_case "answers in pieces" -- -d $D --samples 2 -O analog
    emu_options model=34401A pace=25
    emu_case "real rate" -- -d $D --samples 2 -O analog
    emu_case "real rate, a time limit shorter than the pace" -- -d $D --time 100 -O analog
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $D --scan
}
