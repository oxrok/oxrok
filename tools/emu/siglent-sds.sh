# Siglent SDS scopes (emu/src/models/siglent-sds.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=siglent-sds
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=siglent-sds
EMU_GCOV=siglent-sds
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=siglent-sds
    T=siglent-sds:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $T --show
    emu_case "show usb" --keep-header -- -d $U --show
    emu_case "show channel group" -- -d $T -g CH1 --show
    # An E series on USBTMC: its second read_begin finds nothing, for ever.
    emu_case "frame" --hangs --timeout 12 --no-output -- -d $U -C CH1 --frames 1 -O analog
    # With samples: it asks for 0 bytes at the end, and dies.
    emu_case "frame tcp" --expect-fail -- -d $T -C CH1 --frames 1 -O analog
    emu_options model=SDS1202X
    emu_case "spo frame" --expect-fail -- -d $U -C CH1 --frames 1 -O analog
    # The one block it reads to the end has no samples.
    emu_options model=SDS1202X depth=0
    emu_case "spo empty frames" --no-output -- -d $U --frames 2 -O analog
    # What ends where the reads end is a read error, which it takes for an end.
    emu_options model=SDS1202X depth=400 cut=363
    emu_case "spo cut" -- -d $U -C CH1 --frames 1 -O analog
}
