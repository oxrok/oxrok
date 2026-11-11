# The MIC 98581 and 98583 loggers (emu/src/models/mic-985xx.rs) on the CDC-ACM
# shell. Sourced by tools/diffcli.sh --emu; see docs/USB.md, "Writing a serial
# model".
#
# A capture's readings go by burst= (or a limit that the last of them meets), so
# no count here depends on how long a tool listens. Only "free stream" is not
# held, and it is the one with lengths filtered.
EMU_MODEL=mic-985xx
EMU_CDC=1
EMU_DRIVER="mic-98583 mic-98581"
EMU_FULL=$EMU_DRIVER
EMU_GCOV=mic-985xx
EMU_GCOV_SHARED="src/serial.c src/serial_libsp.c"

emu_conns() {
    S=mic-98583:conn=$EMU_TTY
    T=mic-98581:conn=$EMU_TTY
}

# The readings of these lines, as the bytes of one packet: all of it comes in one
# transfer, so none of this goes by the time. A | is a CR. Keep it within 128
# bytes.
mic_text() {
    emu_hex "$(printf '%s' "$*" | sed 's/|/\\r/g')"
}

emu_cases() {
    emu_conns
    # oxrok's log prefix, on its side only.
    F='s/^oxrok: [a-z]+: oxrok::drivers::mic_985xx: /sr: mic-985xx: /'
    TENTHS="packet_parse divides the field by 10 in integers (protocol.c:83, :87) and throws away the third digit, under the 1 decimal digit its sr_analog_init reports (protocol.c:115); oxrok keeps it. That a logger's third digit is real is an inference: no recording"
    HUP_BUG="receive_data takes a hung up port for a timeout (protocol.c:202-211: revents is not exactly G_IO_IN), so a sample limit is never met; oxrok ends the capture"
    CHANNELS="handle_packet sends both channels whatever is enabled (protocol.c:122-142), and the default output prints them; oxrok sends a channel only if it is on"
    NO_READINGS="sigrok-cli says nothing of a capture that ends with no readings; oxrok warns"
    SIGINT_BUG="sigrok-cli has no SIGINT handler, it dies where oxrok ends the capture"

    # What a scan is: an open and a close, and no word to the logger.
    emu_options burst=0
    emu_case "show 98583" --keep-header -- -d $S --show
    emu_case "show 98581" --driver mic-98581 --keep-header -- -d $T --show
    emu_case "scan 98583" --keep-header -- -d $S --scan
    emu_case "scan 98581" --driver mic-98581 --keep-header -- -d $T --scan
    emu_case "no conn" --keep-header --no-output -- -d mic-98583 --scan
    emu_case "no such node" --keep-header --no-output -- -d mic-98583:conn=/dev/ttyACM97 --scan
    emu_case "scan at any serialcomm" --keep-header -- -d $S:serialcomm=9600/8n1 --scan

    # The keys: two limits to set, nothing to read.
    emu_case "get limit_samples" --expect-fail --stderr -- -d $S --get limit_samples
    emu_case "get conn" --expect-fail --stderr -- -d $S --get conn
    emu_case "set limit_samples" --no-output -- -d $S --set -c limit_samples=3
    emu_case "set samplerate" --expect-fail --stderr -- -d $S --set -c samplerate=1

    # A capture: the command at the first silence, then what comes.
    emu_options burst=1
    emu_case "samples 1" --keep-header -- -d $S --samples 1
    # -c alone is no capture to sigrok-cli: the time limit asks for one, and is not
    # what ends it.
    emu_case "limit_samples" --keep-header -- -d $S -c limit_samples=1 --time 5s
    emu_case "csv" -- -d $S --samples 1 -O csv:header=false
    emu_case "Temperature alone" --oracle-differs "$CHANNELS" --keep-header -- -d $S -C Temperature --samples 1
    emu_case "Humidity alone" --oracle-differs "$CHANNELS" --keep-header -- -d $S -C Humidity --samples 1
    emu_case "Humidity alone in csv" -- -d $S -C Humidity --samples 1 -O csv:header=false
    emu_case "renamed" --keep-header -- -d $S -C Temperature=air,Humidity --samples 1
    emu_case "time holds the burst" --keep-header -- -d $S --time 800ms
    emu_options device=98581 burst=1
    emu_case "98581 samples 1" --driver mic-98581 --keep-header -- -d $T --samples 1
    emu_case "98581 time" --driver mic-98581 --keep-header -- -d $T --time 800ms
    emu_options burst=2
    emu_case "samples 2" --keep-header -- -d $S --samples 2
    # The one free stream. Each read's length is a matter of time; the readings
    # and the count are not, as a period of 1 s outlasts a stall.
    emu_options
    emu_case "free stream" --keep-header --tx-filter 's/^bulk_in 82 [0-9]+ -> .*$/bulk_in 82 STREAM/' -- -d $S --samples 2

    # A time limit: the timer runs before it is looked at, so even one shorter
    # than the first silence sends the command.
    emu_options burst=0
    emu_case "time before the first silence" --no-output -- -d $S --time 50ms
    emu_case "silent at a time limit" --no-output -- -d $S --time 300ms
    emu_case "silent at a time limit warns" --oracle-differs "$NO_READINGS" --stderr --filter "$F" -- -d $S --time 300ms
    emu_options burst=0 ack=$(mic_text 'OK|')
    emu_case "only an answer to the command" --no-output -- -d $S --time 300ms

    # All of a read goes out before the limit is looked at.
    emu_options burst=1 packet=$(mic_text 'v 230 610|v 240 620|v 250 630|')
    emu_case "a read of three at samples 1" --keep-header -- -d $S --samples 1
    emu_options device=98581 burst=1 packet=$(mic_text 'v 230|v 240|')
    emu_case "98581 a read of two at samples 1" --driver mic-98581 --keep-header -- -d $T --samples 1

    # A frame in two reads.
    emu_options burst=2 packet=$(mic_text 'v 230 6'),$(mic_text '10|')
    emu_case "a frame in two reads" --keep-header -- -d $S --samples 1
    emu_options device=98581 burst=2 packet=$(mic_text 'v 2'),$(mic_text '30|')
    emu_case "98581 a frame in two reads" --driver mic-98581 --keep-header -- -d $T --samples 1

    # The values, whole degrees. Then a third digit that is not 0.
    emu_options burst=1 packet=$(mic_text 'v 000 000|v 010 990|v 990 010|v 500 500|')
    emu_case "values" --keep-header -- -d $S --samples 4
    emu_options device=98581 burst=1 packet=$(mic_text 'v 000|v 010|v 500|v 990|')
    emu_case "98581 values" --driver mic-98581 --keep-header -- -d $T --samples 4
    emu_options burst=1 packet=$(mic_text 'v 235 612|v 009 999|v 999 001|')
    emu_case "tenths" --oracle-differs "$TENTHS" --keep-header -- -d $S --samples 3
    emu_options device=98581 burst=1 packet=$(mic_text 'v 235|v 009|v 999|')
    emu_case "98581 tenths" --oracle-differs "$TENTHS" --driver mic-98581 --keep-header -- -d $T --samples 3

    # What is no frame: each byte's check in turn, and a good one to end on.
    emu_options burst=2 packet=$(mic_text 'w 230 610|vx230 610|v x30 610|v 2x0 610|v 23x 610|'),$(mic_text 'v 230x610|v 230 x10|v 230 6x0|v 230 61x|v 230 610x|v 240 620|')
    emu_case "each check of a frame" --keep-header -- -d $S --samples 1
    emu_options device=98581 burst=1 packet=$(mic_text 'w 230|vx230|v x30|v 2x0|v 23x|v 230x|v 240|')
    emu_case "98581 each check of a frame" --driver mic-98581 --keep-header -- -d $T --samples 1
    # A false start: the search goes on from its next byte, not its end.
    emu_options burst=1 packet=$(mic_text 'v 230 v 240 620|')
    emu_case "a frame inside a false start" --keep-header -- -d $S --samples 1
    emu_options device=98581 burst=1 packet=$(mic_text 'v 2v 240|')
    emu_case "98581 a frame inside a false start" --driver mic-98581 --keep-header -- -d $T --samples 1

    # How a temperature below 0 is written is not known: no sign makes a frame, in
    # either tool. A known limit.
    emu_options burst=1 packet=$(mic_text 'v -12 610|v-123 610|v -123 610|v 240 620|')
    emu_case "a sign is no frame" --keep-header -- -d $S --samples 1

    # Bytes that are no reading, with the readings.
    emu_options burst=2 noise=3f3f3f
    emu_case "noise before each reading" --keep-header -- -d $S --samples 2
    emu_options burst=1 ack=$(mic_text 'OK|')
    emu_case "an answer to the command" --keep-header -- -d $S --samples 1
    # The other logger's frames are no frames.
    emu_options burst=1
    emu_case "a 98583 to the 98581's driver" --driver mic-98581 --no-output -- -d $T --time 800ms
    emu_options device=98581 burst=1 packet=$(mic_text 'v 230|v 240|v 250|v 260|')
    emu_case "a 98581 to the 98583's driver" --no-output -- -d $S --time 800ms

    # A logger already sending, slower than the timer: its first silence gets the
    # command after all. (One faster than the timer gets none: no case here, as
    # an open's 16 URBs are offers too, a reading before the open's flush is lost,
    # and what is left of the 100 ms is no margin. The port's unit tests hold it.)
    # 60 offers: an open's 32, and the rest about 20 ms each.
    emu_options running=1 late=60 burst=1
    emu_case "a command to one that is sending" --keep-header -- -d $S --samples 1
    # The command takes effect late: nothing more is asked.
    emu_options late=50 burst=1
    emu_case "a command that takes effect late" --keep-header -- -d $S --samples 1

    # The line.
    emu_options burst=1
    emu_case "one stop bit" --keep-header -- -d $S:serialcomm=38400/8n1 --samples 1
    emu_case "wrong baud hears nothing" --no-output -- -d $S:serialcomm=9600/8n2 --time 400ms
    emu_options stall=line burst=0
    emu_case "line coding stalled" --keep-header -- -d $S --scan
    emu_options stall=lines
    emu_case "control lines stalled" --keep-header -- -d $S --scan

    # An unplug with the tty open: C takes a hang up for a timeout, for as long as
    # a time limit lets it. The mark is a period after the reading.
    emu_options burst=1
    emu_case "unplug at a time limit" --oxrok-fails "the port gone" --unplug model -- -d $S --time 3s
    emu_case "unplug at a sample limit" --oxrok-fails "the port gone" --oracle-bug "$HUP_BUG" --oracle-hangs --unplug model --timeout 5 -- -d $S --samples 5
    emu_case "interrupt" --oracle-bug "$SIGINT_BUG" --interrupt model --timeout 8 -- -d $S --samples 5
    # No case for a hang up before the first silence, where C's write fails and it
    # goes on (protocol.c:29-30): an --unplug out=0:3 reached it, but in 1 of 15
    # runs it beat the open's termios and C failed the open instead.
    emu_options
}
