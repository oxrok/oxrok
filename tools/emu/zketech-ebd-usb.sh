# A ZKETECH EBD-USB+ (emu/src/models/zketech-ebd-usb.rs) on the CDC-ACM shell.
# Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial model".
EMU_MODEL=zketech-ebd-usb
EMU_CDC=1
EMU_DRIVER=zketech-ebd-usb
EMU_GCOV=zketech-ebd-usb
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_cases() {
    emu_conns
    # Scans send 05 and 06 and read one frame.
    emu_options setters=0
    emu_case "scan" --keep-header -- -d $A --scan
    emu_case "show" --keep-header -- -d $A --show
    emu_case "no such node" --keep-header --no-output -- -d zketech-ebd-usb:conn=/dev/ttyACM97 --scan
    emu_case "no conn" --keep-header --no-output -- -d zketech-ebd-usb --scan
    emu_case "show without a device" --keep-header --stderr --expect-fail -- -d zketech-ebd-usb --show
    emu_options setters=0 frames=0
    emu_case "scan silent" --keep-header --stderr -- -d $A --scan
    emu_options setters=0 replies=fa00f8
    emu_case "scan a short frame is a device" --keep-header -- -d $A --scan
    emu_options setters=0 replies=faf8
    emu_case "scan no body" --keep-header --stderr -- -d $A --scan
    emu_options setters=0 replies=55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa55aa period=2
    emu_case "scan garbage" --keep-header --no-output -- -d $A --scan
    # One byte per transfer drives offers fast, so a period is no time at all:
    # the first frame of each start is all there is.
    emu_options setters=0 chunk=1 period=100000
    emu_case "scan one byte per transfer" --keep-header -- -d $A --scan
    # The line.
    emu_options setters=0
    emu_case "serialcomm as the default" --keep-header -- -d $A:serialcomm=9600/8e1 --scan
    emu_case "serialcomm another rate" --keep-header --no-output -- -d $A:serialcomm=19200/8e1 --scan
    emu_case "serialcomm no parity" --keep-header --no-output -- -d $A:serialcomm=9600/8n1 --scan
    emu_case "serialcomm no such thing" --keep-header --no-output --tx-filter '/^ctl_out 21 22 /d' -- -d $A:serialcomm=fast --scan
    emu_options setters=0 uart=19200/8e1
    emu_case "serialcomm as the device is set" --keep-header -- -d $A:serialcomm=19200/8e1 --scan
    # Settings.
    emu_options setters=0
    emu_case "get current_limit" -- -d $A --get current_limit
    emu_case "get uvc_threshold" -- -d $A --get uvc_threshold
    emu_case "get limit_samples" -- -d $A --get limit_samples
    emu_case "set current_limit" --no-output -- -d $A -c current_limit=1.5 --set
    emu_case "set uvc_threshold" --no-output -- -d $A -c uvc_threshold=3 --set
    emu_case "set then read back" -- -d $A -c current_limit=0.3 --get current_limit
    emu_case "set then read back uvc" -- -d $A -c uvc_threshold=3.3 --get uvc_threshold
    emu_case "set current_limit too high" --stderr --expect-fail -- -d $A -c current_limit=4.01 --set
    emu_case "set current_limit negative" --stderr --expect-fail -- -d $A -c current_limit=-1 --set
    emu_case "set current_limit nan" --no-output -- -d $A -c current_limit=nan --set
    emu_case "set uvc_threshold too high" --stderr --expect-fail -- -d $A -c uvc_threshold=21.01 --set
    emu_case "set then read back -0" -- -d $A -c current_limit=-0 --get current_limit
    emu_case "set the most" --no-output -- -d $A -c current_limit=4:uvc_threshold=21 --set
    emu_case "set just over" --stderr --expect-fail -- -d $A -c current_limit=4.001 --set
    emu_case "set in single precision" -- -d $A -c current_limit=4.0000001 --get current_limit
    # Captures with nothing set. Nothing goes to the load: setters=0.
    emu_case "one sample" -- -d $A --samples 1
    emu_case "three samples" -- -d $A --samples 3
    emu_case "three samples csv" -- -d $A --samples 3 -O csv:header=false:time=false
    emu_case "three samples analog" -- -d $A --samples 3 -O analog
    emu_case "one channel" -- -d $A -C I.VBUS --samples 2
    emu_case "one more channel" -- -d $A -C V.D+,V.VBUS --samples 2
    # sigrok-cli reads the dash as a range.
    emu_case "the channel with a dash" --stderr --expect-fail -- -d $A -C V.D- --samples 2
    emu_case "time" --stderr --expect-fail -- -d $A --time 100
    emu_case "frames" --stderr --expect-fail -- -d $A --frames 1
    # sigrok-cli stops a --continuous at the EOF on its stdin.
    emu_case "config limit_samples" -- -d $A -c limit_samples=2 --continuous
    emu_case "continuous" -- -d $A --continuous
    emu_options setters=0 chunk=1 period=100000
    emu_case "capture one byte per transfer" -- -d $A --samples 1
    # The load. A setting reaches it only from the first frame of a capture.
    emu_options
    emu_case "load one sample" -- -d $A -c current_limit=1.5 --samples 1
    emu_case "load two samples" -- -d $A -c current_limit=1.5 --samples 2
    emu_case "load four samples" -- -d $A -c current_limit=1.5:uvc_threshold=3 --samples 4
    emu_case "load off the grid" -- -d $A -c current_limit=0.0015 --samples 4
    emu_case "load the least" -- -d $A -c current_limit=0.001 --samples 2
    emu_case "load below the least" -- -d $A -c current_limit=0.0009 --samples 2
    emu_case "load the most" -- -d $A -c current_limit=4:uvc_threshold=21 --samples 2
    emu_case "load 0.29" -- -d $A -c current_limit=0.29:uvc_threshold=5.1 --samples 3
    emu_case "threshold alone" -- -d $A -c uvc_threshold=3 --samples 3
    # The XOR is fe, and with this D+ the device's own frames stay below f0.
    emu_options dp=1721
    emu_case "a command's checksum at the top" -- -d $A -c current_limit=3.824:uvc_threshold=0.16 --samples 2
    emu_options load=1 limit=1500 cutoff=300
    emu_case "load on, nothing set" -- -d $A --samples 3
    emu_case "load on, the same set" -- -d $A -c current_limit=1.5:uvc_threshold=3 --samples 3
    emu_case "load on, another set" -- -d $A -c current_limit=1:uvc_threshold=3 --samples 3
    emu_case "load on, one sample" -- -d $A -c current_limit=1.5:uvc_threshold=3 --samples 1
    emu_options device=1 load=1
    emu_case "device frames, load on" -- -d $A -c current_limit=1 --samples 3
    emu_options device=1
    emu_case "device frames" -- -d $A -c current_limit=1 --samples 3
    emu_options sag=1000 vbus=5000
    emu_case "a load that falls off starts again" -- -d $A -c current_limit=1:uvc_threshold=3.5 --samples 6
    emu_options deaf=1
    emu_case "deaf repeats" -- -d $A -c current_limit=1 --samples 3

    # What can come. The first frame is the scan's, so only a capture sees the rest.
    G=fa00000014c507a907ad0064000a00001aa1f8
    emu_options replies=$G,fa00000014c507a907ad0064000a00001aa0f8,$G
    emu_case "wrong checksum" -- -d $A --samples 2
    emu_case "wrong checksum says so" --stderr -- -d $A --samples 2
    emu_options replies=$G,fa00f8,$G
    emu_case "short frame" -- -d $A --samples 2
    emu_options replies=$G,fa0a000a0000000000f8,$G
    emu_case "load timer" -- -d $A --samples 2
    emu_options replies=$G,fa01000014c507a907ad0064000a00001aa0f8,$G
    emu_case "unknown type" -- -d $A --samples 2
    emu_options replies=$G,faf8,$G
    emu_case "no body" -- -d $A --samples 2
    emu_case "no body says so" --stderr -- -d $A --samples 2
    emu_options replies=$G,fafa00000014c507a907ad0064000a00001aa1f8
    emu_case "begin twice" -- -d $A --samples 3
    emu_case "begin twice says so" --stderr -- -d $A --samples 3
    emu_options replies=$G,55$G
    emu_case "stray byte" -- -d $A --samples 3
    emu_case "stray byte says so" --stderr -- -d $A --samples 3
    emu_options replies=$G,fa0000000000000000000000000000000000000000000000f8,$G
    emu_case "too long" -- -d $A --samples 3
    emu_case "too long says so" --stderr -- -d $A --samples 3
    # With no fa or f8 past the 19th byte: what follows is read as a frame with
    # no begin, and the next good one is lost with it.
    emu_options replies=$G,fa00000014c507a907ad0064000a00001aa1aa0000000014c507a907ad0064000a00001aa1f8,$G
    emu_case "too long and well formed" -- -d $A --samples 2
    # A checksum the XOR takes to f8 or more. 4955 mV plus the ripple.
    emu_options vbus=4955
    emu_case "a checksum modulo f0" -- -d $A --samples 1
    emu_case "a checksum modulo f0 says so" --stderr -- -d $A --samples 1
    emu_options vbus=4955 raw-checksum=1
    emu_case "a checksum of f8" -- -d $A --samples 1
    emu_case "a checksum of f8 says so" --stderr -- -d $A --samples 1

    # A device that goes quiet ends the capture, with no stop: frames= counts the
    # scan's.
    emu_options frames=1
    emu_case "silent in a capture" --no-output -- -d $A --samples 3
    emu_case "silent in a capture says so" --stderr -- -d $A --samples 3
    emu_options frames=3
    emu_case "silent after two" -- -d $A --samples 3
    emu_case "silent after two says so" --stderr -- -d $A --samples 3
    emu_case "silent with the load on" -- -d $A -c current_limit=1 --samples 3
    emu_options frames=1
    emu_case "silent with continuous" --no-output -- -d $A --continuous

    # The library runs it for longer. Four analog packets a frame.
    emu_options
    emu_case "lib two frames" --libsigrok -- -d $A --continuous --stop-after 8
    emu_case "lib load" --libsigrok -- -d $A -c current_limit=1 --continuous --stop-after 12
    emu_case "lib stop at the header" --libsigrok --no-output -- -d $A --continuous --stop-after 0
    emu_options frames=1
    emu_case "lib silent" --libsigrok --stderr -- -d $A --continuous
    emu_options frames=3
    emu_case "lib silent after two" --libsigrok -- -d $A --continuous

    # A device that leaves: C's read neither ends nor fails.
    emu_options frames=2
    emu_case "unplug in a capture" --unplug model --hangs --timeout 5 --filter '1,4!d' -- -d $A --samples 3
    emu_options frames=0
    emu_case "unplug in the scan" --unplug model --hangs --timeout 5 --keep-header --no-output -- -d $A --scan
    emu_options
    emu_case "unplug in the scan's stop" --unplug out=2:13 --stderr --expect-fail -- -d $A --samples 3
    emu_case "unplug in a capture's start" --unplug out=2:25 --hangs --timeout 5 --no-output -- -d $A --samples 3
    emu_case "unplug in a capture's stop" --unplug out=2:35 --filter '1,4!d' --timeout 5 -- -d $A --samples 1

    # The USB side misbehaving, the shell's options.
    emu_options setters=0 stall=line
    emu_case "line coding stalled" --keep-header --no-output -- -d $A --scan
    emu_options setters=0 stall=lines
    emu_case "control lines stalled" --keep-header -- -d $A --scan
    emu_options setters=0 in-stall=1
    emu_case "a stalled read" --keep-header -- -d $A --scan
    emu_options
}

# See emu_options in tools/diffcli.sh.
emu_conns() {
    A=zketech-ebd-usb:conn=$EMU_TTY
}
