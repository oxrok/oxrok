# Hameg and Rohde & Schwarz scopes (emu/src/models/hameg-hmo.rs), on USBTMC and on TCP.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a SCPI model".
EMU_MODEL=hameg-hmo
EMU_TMC=1
EMU_TCP=1
EMU_DRIVER=hameg-hmo
EMU_FULL=
EMU_GCOV=hameg-hmo
EMU_GCOV_SHARED="src/scpi/scpi.c src/scpi/scpi_usbtmc_libusb.c src/scpi/scpi_tcp.c src/tcp.c"

emu_cases() {
    # A capture's output in few lines: every 64th, and through --digest the hash of all.
    A='1~64!d'
    # By scan: a re-attach moves the address, so conn=usbtmc/$EMU_USB only holds with no
    # options changed.
    U=hameg-hmo
    T=hameg-hmo:conn=tcp-raw/127.0.0.1/$EMU_TCP_PORT

    # The scope, on both carriers.
    emu_case "scan" --keep-header -- -d $U --scan
    emu_case "scan tcp" --keep-header -- -d $T --scan
    emu_case "show" --keep-header -- -d $U --show
    emu_case "show tcp" --keep-header -- -d $T --show
    emu_case "show CH1" -- -d $T -g CH1 --show
    emu_case "show POD1" -- -d $T -g POD1 --show
    emu_case "show POD2" -- -d $T -g POD2 --show

    # Each value.
    for GET_KEY in timebase samplerate num_hdiv horiz_triggerpos triggersource triggerslope triggerpattern highresolution peakdetection limit_frames; do
        emu_case "get $GET_KEY" -- -d $T --get $GET_KEY
    done
    emu_case "get limit_samples is no get" --no-output --expect-fail -- -d $T --get limit_samples
    emu_case "get vdiv" -- -d $T -g CH2 --get vdiv
    emu_case "get num_vdiv" -- -d $T -g CH2 --get num_vdiv
    emu_case "get coupling" -- -d $T -g CH2 --get coupling
    emu_case "get vdiv needs a group" --no-output --expect-fail -- -d $T --get vdiv
    emu_case "get logic_threshold" -- -d $T -g POD2 --get logic_threshold
    emu_case "get custom threshold not in custom mode" --no-output --expect-fail -- -d $T -g POD1 --get logic_threshold_custom
    emu_case "no such group" --expect-fail -- -d $T -g POD3 --show

    # Each setting.
    emu_case "set timebase" --no-output -- -d $U --config timebase=2ms --set
    emu_case "set timebase tcp" --no-output -- -d $T --config timebase=500us --set
    emu_case "set horiz_triggerpos" --no-output -- -d $T --config horiz_triggerpos=0.25 --set
    emu_case "set horiz_triggerpos centre" --no-output -- -d $T --config horiz_triggerpos=0.5 --set
    emu_case "set horiz_triggerpos out of range" --no-output --expect-fail -- -d $T --config horiz_triggerpos=2 --set
    emu_case "set trigger source" --no-output -- -d $T --config triggersource=D3 --set
    emu_case "set trigger slope" --no-output -- -d $T --config triggerslope=NEG --set
    emu_case "set trigger pattern" --no-output -- -d $T --config triggerpattern=10X --set
    emu_case "set trigger pattern too long" --no-output --expect-fail -- -d $T --config triggerpattern=101010101010101010101 --set
    emu_case "set trigger pattern at its longest" --no-output -- -d $T --config triggerpattern=10101010101010101010 --set
    emu_case "set high resolution on" --no-output -- -d $T --config highresolution=on --set
    emu_case "set high resolution off" --no-output -- -d $T --config highresolution=off --set
    emu_case "set peak detection on" --no-output -- -d $T --config peakdetection=on --set
    emu_case "set peak detection off" --no-output -- -d $T --config peakdetection=off --set
    emu_case "set vdiv" --no-output -- -d $T -g CH2 --config vdiv=500mV --set
    emu_case "set vdiv needs a group" --no-output --expect-fail -- -d $T --config vdiv=500mV --set
    emu_case "set vdiv not on a pod" --no-output --expect-fail -- -d $T -g POD1 --config vdiv=500mV --set
    emu_case "set vdiv not in the table" --no-output --expect-fail -- -d $T -g CH1 --config vdiv=20V --set
    emu_case "set coupling" --no-output -- -d $T -g CH1 --config coupling=AC --set
    emu_case "set logic_threshold" --no-output -- -d $T -g POD2 --config logic_threshold=CMOS --set
    emu_case "set custom threshold" --no-output -- -d $T -g POD2 --config logic_threshold_custom=1.5 --set
    emu_case "set custom threshold out of range" --no-output --expect-fail -- -d $T -g POD1 --config logic_threshold_custom=9 --set

    # Captures. Two channels are on at power on, as a scope says at the open.
    emu_case "frame" --keep-header --digest --filter "$A" -- -d $U --frames 1 -O analog
    emu_case "frame tcp" --keep-header --digest --filter "$A" -- -d $T --frames 1 -O analog
    emu_case "one channel off" --digest --filter "$A" -- -d $T -C CH2 --frames 1 -O bits
    emu_case "all analog" --digest --filter "$A" -- -d $T -C CH1,CH2,CH3,CH4 --frames 1 -O bits
    emu_case "frames alone" --digest --filter "$A" -- -d $T -C CH1 --frames 3 -O bits
    emu_case "frames and a high limit" --digest --filter "$A" -- -d $T -C CH1 --frames 2 --samples 5000 -O bits
    emu_case "frames and a low limit" --digest --filter "$A" -- -d $T -C CH1 --frames 2 --samples 10 -O analog
    emu_case "samples" --digest -- -d $T -C CH1 --samples 10 -O analog
    emu_case "samples above a frame" --digest --filter "$A" -- -d $T -C CH1 --samples 5000 -O bits
    emu_case "time" --digest --filter "$A" -- -d $T -C CH1 --time 1ms -O bits
    emu_case "continuous" --no-output --expect-fail -- -d $T -C CH1 --continuous -O bits
    emu_case "pod one" --keep-header --digest --filter "$A" -- -d $U -C D0 --frames 1 -O bits
    emu_case "pod one tcp" --digest --filter "$A" -- -d $T -C D0,D7 --frames 1 -O bits
    emu_case "pod two alone" --digest --filter "$A" -- -d $T -C D8 --frames 1 -O bits
    emu_case "both pods" --keep-header --digest --filter "$A" -- -d $T -C D0,D15 --frames 1 -O bits
    emu_case "both pods samples" -- -d $T -C D0,D8 --samples 8 -O bits
    emu_case "pod samples" -- -d $T -C D0 --samples 8 -O bits
    emu_case "channel and pod" --keep-header --digest --filter "$A" -- -d $T -C CH1,D0 --frames 1 -O bits
    emu_case "channel and pod, analog" --digest --filter "$A" -- -d $T -C CH1,D0 --frames 1 -O bits
    emu_case "CH3 with pod one" --no-output --expect-fail -- -d $T -C CH3,D0 --frames 1 -O bits
    emu_case "CH4 with pod two" --no-output --expect-fail -- -d $T -C CH4,D8 --frames 1 -O bits

    # What the scope says at the open.
    emu_options set=LOG0:STAT=1 set=POD1:STAT=1
    emu_case "on at the open" --digest --filter "$A" -- -d $T --frames 1 -O bits
    emu_options set=PROB1:SET:ATT:UNIT=A
    emu_case "a current probe" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O analog
    emu_options set=POD1:THR=USER2
    emu_case "get custom threshold" -- -d $T -g POD1 --get logic_threshold_custom
    emu_options set=POD1:THR=USER1
    emu_case "threshold USER1" -- -d $T -g POD1 --show
    emu_options set=POD1:THR=USER
    emu_case "threshold USER is USER1" -- -d $T -g POD1 --show
    emu_options set=POD1:THR=BOGUS
    emu_case "threshold nobody knows" --expect-fail -- -d $T --show
    emu_options set=POD1:THR:UDL2=x set=POD1:THR=USER2
    emu_case "a level that is no number" --expect-fail -- -d $T --show
    emu_options set=TRIG:A:SOUR=PATT
    emu_case "the trigger is a pattern" -- -d $T --get triggersource
    emu_options set=ACQ:HRES=AUTO
    emu_case "high resolution is anything but OFF" -- -d $T --get highresolution
    emu_options set=ACQ:PEAK=ON
    emu_case "peak detection is anything but OFF" -- -d $T --get peakdetection
    emu_options set=TIM:DIV=10
    emu_case "ten divisions" -- -d $T --get num_hdiv
    emu_case "ten divisions set position" --no-output -- -d $T --config horiz_triggerpos=0.75 --set
    emu_options set=TIM:POS=3.000E-03
    emu_case "a trigger position" -- -d $T --get horiz_triggerpos
    emu_options set=TRIG:A:PATT:SOUR=10X
    emu_case "a pattern with no quotes" -- -d $T --get triggerpattern
    emu_options 'set=TRIG:A:PATT:SOUR="1010101010101010101010101"'
    emu_case "a long pattern" -- -d $T --get triggerpattern
    emu_options eol=crlf
    emu_case "crlf" --keep-header -- -d $T --show
    for BAD in CHAN1:COUP=BOGUS CHAN1:SCAL=3.000E+00 CHAN1:SCAL=volts CHAN1:POS=x CHAN1:STAT=maybe TIM:SCAL=3.000E-03 TIM:DIV=twelve TIM:POS=x TRIG:A:SOUR=NOPE TRIG:A:EDGE:SLOP=UP LOG0:STAT=maybe POD1:STAT=maybe; do
        emu_options set=$BAD
        emu_case "refused at the open $BAD" --expect-fail -- -d $T --show
    done

    # Blocks in other forms.
    emu_options block-lf=0
    emu_case "no line end after a block" --digest --filter "$A" -- -d $U --frames 1 -O bits
    emu_case "no line end after a block tcp" --digest --filter "$A" -- -d $T --frames 1 -O bits
    emu_options digits=9
    emu_case "nine digits" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O bits
    emu_options odd-at=1
    emu_case "a byte over" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O bits
    emu_options points2=500
    emu_case "pod two shorter" --digest --filter "$A" -- -d $T -C D0,D8 --frames 1 -O bits
    emu_options points2=1010
    emu_case "pod two longer" --digest --filter "$A" -- -d $T -C D0,D8 --frames 1 -O bits

    # How a USBTMC device is found and held.
    emu_reset
    emu_case "usbtmc by address" --keep-header -- -d hameg-hmo:conn=usbtmc/$EMU_USB --scan
    emu_case "usbtmc scan names it" --keep-header -- -d hameg-hmo:conn=usbtmc --scan
    emu_case "usbtmc kernel driver bound" --bound usbtmc --tx-filter '/^ctl_in 21 07 /d' --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O bits
    emu_case "tcp nothing listens" --no-output -- -d hameg-hmo:conn=tcp-raw/127.0.0.1/1 --scan
    emu_options remote=1
    emu_case "remote control" --keep-header -- -d $U --scan
    emu_case "remote control in a frame" --digest --filter "$A" -- -d $U -C CH1 --frames 1 -O bits

    # Who answers.
    emu_options idn=Acme,HMO3054,1,2
    emu_case "scan another maker" --no-output -- -d $T --scan
    emu_options idn=HAMEG,HMO9999,1,2
    emu_case "scan another model" --no-output -- -d $T --scan
    emu_options idn=HAMEG,hmo3054,1,2
    emu_case "scan a name in lower case" --no-output -- -d $T --scan
    emu_options idn=HAMEG,HMO3054,1.0
    emu_case "scan with no serial" --keep-header -- -d $T --scan
    emu_options idn=short
    emu_case "scan with no idn to speak of" --no-output -- -d $U --scan

    # Silence and lateness at the open and in a setting.
    emu_options mute-at=1
    emu_case "scan silent" --no-output -- -d $U --scan
    emu_options silent-at=1
    emu_case "scan silent once" --no-output -- -d $U --scan
    emu_options mute-at=3
    emu_case "show goes silent" --expect-fail -- -d $U --show
    emu_options late-at=3:5
    emu_case "show late" --keep-header -- -d $U --show
    emu_case "show late tcp" --keep-header -- -d $T --show
    emu_options opc-late=20
    emu_case "opc says 0 a while" --no-output -- -d $T --config triggerslope=NEG --set

    # Blocks that don't come, or not whole. The scan and the open take 50 queries here, so
    # the first request is the 51st.
    emu_options late-at=51:5
    emu_case "block late" --digest --filter "$A" -- -d $T --frames 1 -O bits
    emu_case "block late usb" --digest --filter "$A" -- -d $U --frames 1 -O bits
    emu_options odd-at=2
    emu_case "a byte over in the second channel" --digest --filter "$A" -- -d $T --frames 1 -O bits

    # The other series.
    for SCOPE in HMO722 HMO1002 RTC1002 HMO3032 RTB2002 RTB2004 RTM3002 RTM3004 RTA4004 RTH1002 RTH1004; do
        emu_options model=$SCOPE
        emu_case "$SCOPE show" --keep-header -- -d $T --show
        emu_case "$SCOPE POD1" -- -d $T -g POD1 --show
    done
    emu_options model=HMO1002
    emu_case "hmo1002 frame" --digest --filter "$A" -- -d $T -C CH2,D0 --frames 1 -O analog
    emu_case "hmo1002 set custom" --no-output -- -d $T -g POD1 --config logic_threshold_custom=2.5 --set
    emu_case "hmo1002 one pod" --expect-fail -- -d $T -g POD2 --show
    emu_options model=RTB2004
    emu_case "rtb2004 CH1 and both pods" --digest --filter "$A" -- -d $T -C CH1,D0,D15 --frames 1 -O bits
    emu_case "rtb2004 POD2" -- -d $T -g POD2 --show
    emu_case "rtb2004 set threshold" --no-output -- -d $T -g POD2 --config logic_threshold=ECL --set
    emu_case "rtb2004 set custom" --no-output -- -d $T -g POD2 --config logic_threshold_custom=1.5 --set
    emu_case "rtb2004 set 50 V" --no-output -- -d $T -g CH1 --config vdiv=50V --set
    emu_case "rtb2004 set coupling refused" --no-output --expect-fail -- -d $T -g CH1 --config coupling=AC --set
    emu_options model=RTB2004 set=DIG8:TECH=MAN
    emu_case "rtb2004 custom threshold" -- -d $T -g POD2 --get logic_threshold_custom
    emu_options model=RTH1002
    emu_case "rth1002 frame" --digest --filter "$A" -- -d $T -C CH1,D3 --frames 1 -O bits
    emu_case "rth1002 set threshold" --no-output -- -d $T -g POD1 --config logic_threshold=USER2 --set
    emu_options model=RTH1002 set=DIG1:TECH=USER2
    emu_case "rth1002 show custom" -- -d $T -g POD1 --show
    emu_options model=HMO1024
    emu_case "compact show" --keep-header -- -d $T --show
    emu_case "compact get timebase" -- -d $T --get timebase
    emu_case "compact set timebase" --no-output -- -d $T --config timebase=2ms --set
    emu_case "compact frame" --digest --filter "$A" -- -d $T -C CH1 --frames 1 -O bits
    emu_case "compact has no 1 ns" --no-output --expect-fail -- -d $T --config timebase=1ns --set
    emu_options
}
