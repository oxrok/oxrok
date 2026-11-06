# Rigol and Agilent scopes (emu/src/models/rigol-ds.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model". This
# is the template for a SCPI driver's scenario.
EMU_MODEL=rigol-ds
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=rigol-ds
EMU_FULL=$EMU_DRIVER
EMU_GCOV=rigol-ds
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    # A capture's output in few lines: every 64th, and through --digest the hash
    # of all of it.
    A='1~64!d'
    # By scan: a re-attach moves the address, so conn=usbtmc/$EMU_USB only holds
    # with no options changed (see "usbtmc by address").
    U=rigol-ds
    T=rigol-ds:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT
    R=rigol-ds:conn=tcp-rigol/127.0.0.1/$EMU_TCP_PORT

    SIGPIPE="sr_tcp_write_bytes is a plain send() (tcp.c) and nothing in libsigrok or sigrok-cli ignores SIGPIPE, so a write after the instrument closed kills sigrok-cli (exit 141) in the middle of its output; oxrok's write returns EPIPE and the capture fails"
    TCP_BLOCKS="scpi_tcp_raw_read_data has sr_tcp_read_bytes block (nonblocking FALSE, scpi_tcp.c), and its recv() has no timeout, so sr_scpi_get_string's 1 s (scpi.c) never runs and a silent instrument holds sigrok-cli for ever; oxrok's read has the timeout"
    TCP_EOF="sr_tcp_read_bytes returns the 0 of a closed connection as a read of nothing (tcp.c), and neither scpi_tcp_raw_read_data nor rigol_ds_receive takes it for an end, so the capture goes on reading for ever; oxrok says the instrument closed the connection"
    NEVER_ENDS="rigol_ds_receive returns TRUE on each failed step (protocol.c) and its source stays, so with the device gone it tries again for ever and never stops the capture; oxrok's capture fails"
    TRIGGERPOS="api.c's config_set multiplies by num_timebases where the seconds want the horizontal divisions (docs/DEVICES.md, Rigol): C sends another offset, oxrok keeps its choice"

    # A DS1102E, the one with a recording: protocol 2, no line ends, no *OPC?.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show channel group" -- -d $U -g CH1 --show
    emu_case "get timebase" -- -d $T --get timebase
    emu_case "get vdiv" -- -d $T -g CH2 --get vdiv
    emu_case "get coupling" -- -d $T -g CH2 --get coupling
    emu_case "get samplerate" -- -d $T --get samplerate
    emu_case "get triggerslope" -- -d $T --get triggerslope
    emu_case "set timebase" --no-output -- -d $U --config timebase=2ms --set
    emu_case "set coupling tcp" --no-output -- -d $T -g CH1 --config coupling=AC --set
    emu_case "set probe factor" --no-output -- -d $T -g CH1 --config probe_factor=10 --set
    emu_case "set vdiv" --no-output -- -d $T -g CH2 --config vdiv=500mV --set
    emu_case "set trigger source" --no-output -- -d $T --config triggersource=CH2 --set
    emu_case "set trigger slope" --no-output -- -d $T --config triggerslope=f --set
    emu_case "set trigger level" --no-output -- -d $T --config triggerlevel=0.5 --set
    emu_case "set horiz_triggerpos" --no-output --oracle-differs "$TRIGGERPOS" -- -d $T --config horiz_triggerpos=0.25 --set
    emu_case "frame" --record --digest --filter "$A" -- -d $U --frames 1 -O analog
    emu_case "frame tcp" --digest --filter "$A" -- -d $T --frames 1 -O analog
    emu_case "two frames of one channel" --digest --filter "$A" -- -d $T -C CH2 --frames 2 -O analog
    emu_case "a slow timebase polls the trigger" --digest --filter "$A" -- -d $T -c timebase=100ms -C CH1 --frames 2 -O analog
    # sigrok-cli's logger exits at the critical, before the close it would send.
    NOCLOSE='s/^tcp_out ([0-9a-f]*)3a4b45593a4c4f434b2044495341424c450a -> [0-9]+$/tcp_out \1/
s/^tcp_out ([0-9a-f]+) -> [0-9]+$/tcp_out \1/'
    emu_case "memory has no samplerate" --no-output --expect-fail --tx-filter "$NOCLOSE" --oxrok-fails "below V3" -- -d $T -c data_source=Memory --frames 1 -O analog

    # How a USBTMC device is found and held.
    emu_reset
    emu_case "usbtmc by address" --keep-header -- -d rigol-ds:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header -- -d rigol-ds:conn=usbtmc --scan
    # The kernel's driver on it: each tool detaches it and gives it back, and its
    # probe at that asks for the capabilities, which a kernel with no usbtmc doesn't,
    # so those lines go. With the module this proves a tool gets through a driver that
    # holds the interface; without it, only a note says so, and it proves nothing more
    # than the plain frame.
    emu_case "usbtmc kernel driver bound" --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' --digest --filter "$A" -- -d $U --frames 1 -O analog
    emu_case "tcp nothing listens" --no-output -- -d rigol-ds:conn=tcp-raw/127.0.0.1/1 --scan

    # Series. A DS1102E before 0.2.4 and a VS5000 send bare bytes.
    emu_options firmware=00.02.03
    emu_case "old firmware frame" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options model=VS5102
    emu_case "vs5102 frame" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options model=DSO1052B
    emu_case "dso1052b probe says x" -- -d $T -g CH1 --get probe_factor
    emu_options model=DS1054Z
    emu_case "ds1054z show" --keep-header -- -d $T --show
    emu_case "ds1054z frame" --digest --filter "$A" -- -d $T -C CH2,CH3 --frames 1 -O analog
    emu_options model=MSO7034A
    emu_case "mso7034a show" --keep-header -- -d $T --show
    emu_case "mso7034a set horiz_triggerpos" --no-output --oracle-differs "$TRIGGERPOS" -- -d $T --config horiz_triggerpos=0.75 --set
    emu_options model=MSO5074
    emu_case "mso5074 show" --keep-header -- -d $T --show
    emu_case "mso5074 frame" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options model=DS4024
    emu_case "ds4024 memory" --digest --filter "$A" -- -d $T -C CH4 -c data_source=Memory --frames 1 -O analog

    # A DS2072: protocol 3, *OPC? after each setting, #9 blocks with a line end.
    emu_options model=DS2072
    emu_case "ds2072 show" --keep-header -- -d $T --show
    emu_case "ds2072 frame" --digest --filter "$A" -- -d $U --frames 1 -O analog
    emu_case "ds2072 frame tcp" --digest --filter "$A" -- -d $T --frames 1 -O analog
    emu_case "ds2072 set level" --no-output -- -d $T --config triggerlevel=1.5 --set
    emu_case "ds2072 memory" --digest --filter "$A" -- -d $T -C CH1 -c data_source=Memory --frames 1 -O analog
    emu_case "ds2072 segmented" --digest --filter "$A" -- -d $T -C CH1 -c data_source=Segmented --frames 2 -O analog
    emu_options model=DS2072 framing=rigol
    emu_case "ds2072 frame tcp-rigol" --digest --filter "$A" -- -d $R -C CH1 --frames 1 -O analog
    emu_options model=DS2072 block=5000 busy=2
    emu_case "ds2072 memory in blocks" --digest --filter "$A" -- -d $T -C CH1 -c data_source=Memory --frames 1 -O analog
    emu_options model=DS2072 single-fail=1
    emu_case "ds2072 single shot fails once" --digest --filter "$A" -- -d $T -C CH1 -c data_source=Memory --frames 1 -O analog
    emu_options model=DS2072 short-at=1
    emu_case "ds2072 short block" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options model=DS2072 opc-late=20
    emu_case "ds2072 opc late" --no-output -- -d $T --config triggerlevel=1.5 --set

    # The shell's faults: what libsigrok's USBTMC layer takes and what it refuses.
    emu_options model=DS2072 empty-at=1
    emu_case "usbtmc empty transfer" --keep-header -- -d $U --scan
    emu_options model=DS2072 message=20
    emu_case "usbtmc message in pieces" --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O analog
    emu_options model=DS2072 stall=caps
    emu_case "usbtmc capabilities stalled" --keep-header -- -d $U --scan
    emu_options model=DS2072 bad-tag-at=1
    emu_case "usbtmc wrong tag" --keep-header -- -d $U --scan
    emu_options model=DS2072 in-stall=1
    emu_case "usbtmc bulk in stalled" --expect-fail -- -d $U --show

    # A DS1000Z's id is on no list, so remote control goes on the wire: REN_CONTROL and
    # LOCAL_LOCKOUT at each open, GO_TO_LOCAL at each close. sigrok-cli skips its close
    # at an error, so there GO_TO_LOCAL goes from the comparison.
    emu_options model=DS1054Z
    emu_case "remote control" --keep-header -- -d $U --scan
    emu_case "remote control in a frame" --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O analog
    emu_options model=DS1054Z stall=ren
    emu_case "remote control stalled" --keep-header -- -d $U --scan
    emu_options model=DS1054Z stall=llo
    emu_case "local lockout stalled" --keep-header -- -d $U --scan
    emu_options model=DS1054Z mute-at=3
    emu_case "remote control when it goes silent" --expect-fail --tx-filter '/^ctl_in 21 a1 /d' -- -d $U --show

    # What the instrument answers, and when.
    emu_options idn=Acme,X,1,2
    emu_case "scan another instrument" --no-output -- -d $U --scan
    emu_case "scan another instrument tcp" --no-output -- -d $T --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $U --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_case "scan silent tcp" --no-output --oracle-bug "$TCP_BLOCKS" --oracle-hangs --timeout 4 -- -d $T --scan
    emu_options silent-at=1
    emu_case "scan answers the second time" --keep-header -- -d $U --scan
    emu_options mute-at=3
    emu_case "show goes silent" --expect-fail -- -d $U --show
    emu_options model=DS2072 late-at=1:5
    emu_case "scan late" --keep-header -- -d $U --scan
    emu_case "scan late tcp" --keep-header -- -d $T --scan

    # An unplug. On TCP the instrument closes the connection.
    emu_options
    # After the readback, so the writes of the capture's start come next: C dies.
    emu_case "unplug before a capture tcp" --digest --filter "$A" --oracle-bug "$SIGPIPE" --unplug tcp-in=16 --oxrok-fails "unplug" -- -d $T --frames 2 -O analog
    # After a frame's data: C reads 0 bytes for ever, which is no error to it.
    emu_case "unplug in a frame tcp" --digest --filter "$A" --oracle-bug "$TCP_EOF" --oracle-hangs --timeout 4 --unplug tcp-in=18 --oxrok-fails "unplug" -- -d $T --frames 2 -O analog
    emu_case "unplug in a frame" --digest --filter "$A" --oracle-bug "$NEVER_ENDS" --oracle-hangs --timeout 6 --unplug in=2:18 --oxrok-fails "unplug" -- -d $U --frames 2 -O analog
}
