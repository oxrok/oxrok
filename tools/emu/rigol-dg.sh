# Rigol DG function generators (emu/src/models/rigol-dg.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=rigol-dg
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=rigol-dg
EMU_GCOV=rigol-dg
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    U=rigol-dg
    T=rigol-dg:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT


    # A DG1062Z: scan, show, on both carriers.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show channel 1" -- -d $U -g 1 --show
    emu_case "show channel 2 tcp" -- -d $T -g 2 --show
    emu_case "get limit_samples" -- -d $T --get limit_samples
    emu_case "usbtmc by address" --keep-header -- -d rigol-dg:conn=usbtmc/$EMU_USB --scan
    emu_case "tcp nothing listens" --no-output -- -d rigol-dg:conn=tcp-raw/127.0.0.1/1 --scan

    # What a channel says.
    for key in enabled pattern output_frequency amplitude offset phase; do
        emu_case "get $key" -- -d $T -g 1 --get $key
    done
    emu_case "get enabled usb" -- -d $U -g 2 --get enabled
    emu_case "get phase channel 2" -- -d $T -g 2 --get phase
    emu_case "get duty on a sine" --no-output --expect-fail -- -d $T -g 1 --get output_duty_cycle
    emu_options wave=SQU wave2=PULSE
    emu_case "get duty on a square" -- -d $T -g 1 --get output_duty_cycle
    emu_case "get duty on a pulse" -- -d $T -g 2 --get output_duty_cycle
    emu_case "show square" -- -d $T -g 1 --show
    emu_options wave=NOISE wave2=DC
    emu_case "show noise" -- -d $T -g 1 --show
    emu_case "show dc" -- -d $T -g 2 --show
    emu_case "get frequency of dc" -- -d $T -g 2 --get output_frequency

    # What a setting sends. Each reads the state first.
    emu_options
    emu_case "set enabled on" --no-output -- -d $T -g 1 --config enabled=on --set
    emu_case "set enabled off" --no-output -- -d $U -g 2 --config enabled=off --set
    emu_case "set pattern" --no-output -- -d $T -g 1 --config pattern=Square --set
    emu_case "set pattern in any case" --no-output -- -d $T -g 1 --config pattern=pULSE --set
    emu_case "set pattern unknown" --no-output --expect-fail -- -d $T -g 1 --config pattern=Triangle --set
    emu_case "set frequency" --no-output -- -d $T -g 1 --config output_frequency=2000.5 --set
    emu_case "set amplitude" --no-output -- -d $T -g 2 --config amplitude=1.5 --set
    emu_case "set offset" --no-output -- -d $T -g 1 --config offset=-0.25 --set
    emu_case "set phase" --no-output -- -d $T -g 1 --config phase=90 --set
    emu_case "set duty on a sine" --no-output --expect-fail -- -d $T -g 1 --config output_duty_cycle=25 --set
    emu_options wave=SQU wave2=PULSE
    emu_case "set duty on a square" --no-output -- -d $T -g 1 --config output_duty_cycle=25 --set
    emu_case "set duty on a pulse" --no-output -- -d $T -g 2 --config output_duty_cycle=75 --set
    emu_options wave=NOISE wave2=DC
    emu_case "set frequency on noise" --no-output --expect-fail -- -d $T -g 1 --config output_frequency=1000 --set
    emu_case "set phase on noise" --no-output --expect-fail -- -d $T -g 1 --config phase=10 --set
    emu_case "set amplitude on dc" --no-output --expect-fail -- -d $T -g 2 --config amplitude=1 --set
    emu_case "set offset on dc" --no-output -- -d $T -g 2 --config offset=1 --set
    emu_options
    # Refused by the unit, in its queue: neither tool reads it.
    emu_case "set frequency the unit refuses" --no-output -- -d $T -g 1 --config output_frequency=100000000 --set
    emu_case "set several in order" --no-output -- -d $T -g 1 --config output_frequency=2000 --config amplitude=1 --config enabled=on --set

    # A capture.
    emu_case "two frames" -- -d $U --samples 2 -O analog
    emu_case "two frames tcp" -- -d $T --samples 2 -O analog
    emu_case "a frame with the header" --keep-header -- -d $T --samples 1 -O analog
    emu_case "only the counter's frequency" -- -d $T -C FREQ1 --samples 1 -O analog
    emu_case "only a generator's channel" -- -d $T -C CH1 --samples 1 -O analog
    emu_case "time shorter than the poll" -- -d $T --time 20 -O analog
    emu_options counter=RUN
    emu_case "counter already running" -- -d $T --samples 2 -O analog
    emu_options counter=STOP
    emu_case "counter stopped" -- -d $T --samples 1 -O analog
    emu_options model=DG1022Z
    emu_case "dg1022z capture" -- -d $T --samples 1 -O analog
    # Their counter takes a second each way, and channel 2 stays as it is: api.c:902-903
    # makes 2 & 1 of the quirk, so that its check never holds.
    emu_options model=DG811
    emu_case "dg811 capture" -- -d $T --samples 1 -O analog
    emu_options model=DG812 output2=1
    emu_case "dg812 capture with channel 2 on" -- -d $T --samples 1 -O analog
    emu_case "dg812 get enabled" -- -d $T -g 2 --get enabled
    emu_options model=DG992 counter=RUN
    emu_case "dg992 counter running" -- -d $T --samples 1 -O analog

    # Models.
    for m in DG1062Z DG1032Z DG1022Z; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "frequency range $m" -- -d $T -g 1 --show
    done
    for m in DG811 DG812 DG821 DG822 DG831 DG832; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "frequency range $m" -- -d $T -g 1 --show
    done
    for m in DG952 DG972 DG992; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
        emu_case "frequency range $m" -- -d $T -g 1 --show
    done
    for m in MSO5354 MSO5204 MSO5104 MSO5102 MSO5074 MSO5072; do
        emu_options model=$m
        emu_case "scan $m" --keep-header -- -d $T --scan
    done

    # An MSO5000's generator: another table, no counter.
    emu_options model=MSO5074
    emu_case "mso show" --keep-header -- -d $T --show
    emu_case "mso show channel 1" -- -d $T -g 1 --show
    emu_case "mso get enabled" -- -d $T -g 1 --get enabled
    emu_case "mso get frequency" -- -d $T -g 2 --get output_frequency
    emu_case "mso set enabled" --no-output -- -d $T -g 1 --config enabled=on --set
    emu_case "mso set pattern" --no-output -- -d $T -g 1 --config pattern=ExpRise --set
    emu_case "mso set frequency" --no-output -- -d $T -g 1 --config output_frequency=500 --set
    emu_case "mso capture with no key for it" --no-output --expect-fail -- -d $T --samples 1 -O analog
    emu_options model=MSO5074 wave=PULS
    emu_case "mso get duty on a pulse" -- -d $T -g 1 --get output_duty_cycle
    emu_case "mso set duty on a pulse" --no-output -- -d $T -g 1 --config output_duty_cycle=30 --set
    emu_options model=MSO5074 wave=SQU
    emu_case "mso get duty on a square" --no-output --expect-fail -- -d $T -g 1 --get output_duty_cycle
    emu_case "mso set duty on a square" --no-output --expect-fail -- -d $T -g 1 --config output_duty_cycle=30 --set
    emu_options model=MSO5074 wave=SINC
    emu_case "mso sinc" -- -d $T -g 1 --get pattern
    emu_case "mso frequency range of sinc" -- -d $T -g 1 --show

    # What the instrument says at a scan.
    emu_options 'say=*IDN?=RIGOL~TECHNOLOGIES,dg1062z,X,1'
    emu_case "scan in another case" --keep-header -- -d $T --scan
    emu_options 'say=*IDN?=Rigol~Technologies,DG1062Z,1'
    emu_case "scan three fields" --keep-header -- -d $T --scan
    emu_options 'say=*IDN?=Rigol~Technologies,DG1062'
    emu_case "scan two fields" --no-output -- -d $T --scan
    emu_options idn=Acme,X,1,2
    emu_case "scan another instrument" --no-output -- -d $U --scan
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options silent-at=1
    emu_case "scan silent once" --no-output -- -d $U --scan
    emu_options opc-late=3
    emu_case "opc says 0 three times" --keep-header -- -d $U --scan
    emu_options remote=1
    emu_case "remote control" --keep-header -- -d $U --scan

    # A channel's state, as the instrument may say it.
    emu_options 'say=SOUR1:APPL?=SIN,1.000000E+03,5.000000E+00,0.000000E+00,0.000000E+00'
    emu_case "apply with no quotes" -- -d $T -g 1 --get output_frequency
    emu_options 'say=SOUR1:APPL?="SINE,1.5E+03,2,3,4,5"'
    emu_case "apply long" -- -d $T -g 1 --get phase
    emu_options 'say=SOUR1:APPL?="FOO,1,2,3,4"'
    emu_case "apply unknown waveform" --no-output --expect-fail -- -d $T -g 1 --get output_frequency
    emu_case "set on an unknown waveform" --no-output --expect-fail -- -d $T -g 1 --config output_frequency=1 --set
    emu_options 'say=SOUR1:APPL?="SIN,1000"'
    emu_case "apply short" -- -d $T -g 1 --get amplitude
    emu_case "apply short the field it has" -- -d $T -g 1 --get output_frequency
    emu_options 'say=SOUR1:APPL?="SIN,abc,2,3,4"'
    emu_case "apply no number" -- -d $T -g 1 --get output_frequency
    emu_options 'say=SOUR1:APPL?="SIN,1.5xyz,2,3,4"'
    emu_case "apply half a number" -- -d $T -g 1 --get output_frequency
    emu_options 'say=OUTP1:STATE?=maybe'
    emu_case "enabled says neither" --no-output --expect-fail -- -d $T -g 1 --get enabled
    emu_options 'say=SOUR1:FUNC:SQU:DCYC?=abc' wave=SQU
    emu_case "duty no number" --no-output --expect-fail -- -d $T -g 1 --get output_duty_cycle
    emu_options silent-at=4
    emu_case "apply silent" --no-output --expect-fail -- -d $U -g 1 --get output_frequency
    emu_options model=MSO5074 'say=SOUR1:FREQ?=abc'
    emu_case "mso frequency no number" -- -d $T -g 1 --get output_frequency
    emu_options model=MSO5074 'say=SOUR1:FUNC?=FOO'
    emu_case "mso unknown waveform" --no-output --expect-fail -- -d $T -g 1 --get pattern
    emu_options model=MSO5074 silent-at=5
    emu_case "mso frequency silent" --no-output --expect-fail -- -d $U -g 1 --get output_frequency

    # A capture's faults. Queries: 1 *IDN?, 2 *OPC?, 3 COUN:STAT?, 4 *OPC?, 5 *OPC?,
    # 6 COUN:MEAS?, 7 *OPC?, 8 COUN:MEAS?...
    emu_options 'say=COUN:STAT?=NOPE'
    emu_case "counter says something else" -- -d $T --samples 1 -O analog
    emu_options 'play=1,2,3'
    emu_case "measure short" -- -d $T --samples 1 -O analog
    emu_options 'play=;'
    emu_case "measure empty" -- -d $T --samples 1 -O analog
    emu_options 'play=1,2,3,4,5,6'
    emu_case "measure with six" -- -d $T --samples 1 -O analog
    emu_options 'play=a,b,c,d,e'
    emu_case "measure no number" -- -d $T --samples 1 -O analog
    emu_options silent-at=3
    emu_case "counter state silent" --no-output --expect-fail -- -d $U --samples 1 -O analog
    emu_options silent-at=6
    emu_case "first measure silent" --no-output -- -d $U --samples 1 -O analog
    emu_options silent-at=7
    emu_case "an opc silent in a capture" -- -d $U --samples 2 -O analog
    emu_options silent-at=8
    emu_case "second measure silent" -- -d $U --samples 2 -O analog
    emu_options late-at=8:80
    emu_case "second measure late tcp" -- -d $T --samples 3 -O analog
    emu_options opc-late=3
    emu_case "capture opc says 0" -- -d $T --samples 1 -O analog

    # Gone.
    emu_options
    emu_case "unplug in a capture" --unplug in=2:12 -- -d $U --samples 100 -O analog
}
