# Native decoder checklist

Status: 2026-11-07. Reference: libsigrokdecode 0.5.3, the 111
`decoders/*/pd.py` directories installed by Homebrew, which match the
`libsigrokdecode-0.5.3` tag exactly (`common/` is not a decoder).

This is the working checklist for porting every Python decoder to a
native `DecoderDef`. A decoder is native only after its `DecoderInfo`
and every `Output` match the Python decoder on the same input; the
status column says which ones have that proof.

## Summary

| | Count |
|---|---:|
| Python decoders in 0.5.3 | 111 |
| Logic-input decoders | 60 |
| Stacked decoders | 51 |
| Inputless / unclear | 0 |
| Native now | 64 (`i2c`, `spi`, `uart`, `can`, `mdio`, `jtag`, `swd`, `onewire_link`, `ir_nec`, `usb_signalling`, `usb_power_delivery`, `lpc`, `z80`, `parallel`, `ps2`, `lin`, `modbus`, `midi`, `avr_isp`, `spiflash`, `i2cfilter`, `i2cdemux`, `eeprom24xx`, `gpib`, `ieee488`, `counter`, `guess_bitrate`, `jitter`, `pwm`, `timing`, `graycode`, `morse`, `miller`, `rc_encode`, `ir_rc5`, `ir_rc6`, `cec`, `dali`, `dmx512`, `stepper_motor`, `wiegand`, `em4100`, `em4305`, `sdcard_sd`, `st7735`, `rgb_led_ws281x`, `sda2506`, `seven_segment`, `dcf77`, `spdif`, `i2s`, `tdm_audio`, `t55xx`, `am230x`, `dsi`, `qi`, `flexray`, `iec`, `maple_bus`, `ook`, `signature`, `tlc5620`, `microwire`, `mcs48`) |
| Native and parity-tested | 64 |
| Pending | 47 |
| Exceptions | 1 (documented `swd` divergence) |
| Marked deprecated by 0.5.3 | 0 |

libsigrokdecode 0.5.3 has no deprecation flag in the decoder API: no
`pd.py` sets one and the C decoder object has no such field. The count
of 111 is why `docs/DECODERS.md` and the roadmap saying 112 should be
corrected when those files are next touched; the extra name in the
directory listing is `common/`, a helper package shared by decoders.

## Classification

- **Logic input** (60): `inputs = ['logic']`. These read samples from
  the `Stack` and use `wait`, `samplenum`, `matched` and `has_channel`.
- **Stacked** (51): `inputs` name one or more decoder outputs
  (`i2c`, `spi`, `uart`, `onewire_network`, `jtag`, `ook`, `mdio`,
  `microwire`, `onewire_link`, `usb_signalling`, `usb_packet`). They
  implement `decode_item` and must pass `put_python` items exactly.
- **Generates data** (0): no 0.5.3 decoder has an empty `inputs` list
  and produces data without a lower layer. Everything either reads
  logic or stacks on another decoder.
- **Unclear** (0): all 111 import and expose the API-v3 attributes
  under a stub `sigrokdecode` module.

## Parity harness

`tests/decoder_parity.rs` and `tests/support/mod.rs` run the native and
Python `DecoderDef` for one id in process, with the same `Config` and
`Stack`, over the same capture. The capture is loaded through oxrok
itself (`.vcd` through the input module, `.sr` through `SessionReader`),
so the native and Python sides see identical packed samples, channel
names and sample rate. The harness compares the complete `Output`
sequence: annotation instance, span, class and texts; binary payload;
`put_python` item; metadata name and value; and `End`, in order.

The Python host maps `bytearray` to `Item::Bytes` and snapshots lists
and tuples when an item is emitted. This is a host bug fix agreed during
the IEEE488 port: without it, an item kept growing after emission and
the comparison was non-deterministic. Python-to-Python chains receive an
equal list, not the same object.

The Python host keeps non-value objects as `Item::Opaque` so a Python
decoder above gets the same object. For comparison only, `PyItem::to_item`
and `RpItem::to_item` turn lists and tuples into `Item::List` recursively;
the host’s decoding behaviour is unchanged. `tests/support` exposes
`python(id)`, which finds the Python definition even when a native
decoder has the same id, so a test always has a reference to force.

The tests are gated on `any(feature = "python", feature = "rustpython")`;
a plain `cargo test` compiles them away. Run them with either backend:

```sh
cargo test --features python --test decoder_parity
cargo test --features rustpython --test decoder_parity
```

## Pilot results

The three existing native decoders pass parity on the VCD fixtures and
on the bench `.sr` captures:

| Decoder | Capture | Cases | Result |
|---|---|---|---|
| `uart` | `uart-hi.vcd`, `uart-esp32.sr` | default; `format`, `bit_order`, `data_bits`, `parity`, `stop_bits`, `invert_rx`; 921600 bench capture | pass, 52 corpus cases |
| `i2c` | `i2c-write-read.vcd`, `i2c-esp32.sr` | default; `address_format=shifted/unshifted`; bench capture | pass, 9 corpus cases |
| `spi` | `spi-mode0.vcd`, `spi-esp32.sr` | default; `cpol`/`cpha`, `bitorder`, `wordsize`, `cs_polarity`; bench capture | pass, 13 corpus cases |

The harness found one real mismatch before it went green: with
`invert_rx=yes`, Python’s `inspect_sample` does `signal = not signal`,
so the `put_python` STARTBIT/PARITYBIT/STOPBIT and per-bit items carry
`bool`, while the native decoder emitted `int`. Native `uart` now uses
`Item::Bool` on inverted lines and `Item::Int` otherwise, as Python
does; annotations and CLI output are unchanged. This is the kind of
wire-format difference the harness is meant to catch.

## Reference corpus

The recordings are in `sigrokproject/sigrok-dumps` (about 80 MB), not
in `sigrok-test`: `sigrok-test/decoder/test/<decoder>/test.conf` has the
test definitions and expected `.output`, `.python` and binary goldens,
and its `input` lines name `.sr` files in `sigrok-dumps`. The checkout
used here has 118 decoder directories and 594 cases, more than the 111
0.5.3 decoders because it also covers newer decoders such as `ir_irmp`
and `caliper`.

The corpus runner reads `test.conf`, maps `channel`/`option` pairs to
`Config`, builds the whole `stack` for stacked cases, loads the named
`.sr`, and compares the native chain with the Python chain. It does not
need the `.output` goldens. It is opt-in with `OXROK_CORPUS=1`; cases
whose lower decoder is not native yet, or whose input the dump is
missing, are skipped and counted. `OXROK_CORPUS_DECODER=<id>` limits a
run to one decoder. The existing `tools/diffcli.sh` stays the
end-to-end check against sigrok-cli.

## Batch order

The suggested order puts `lin`, `modbus`, `avr_isp`, and the i2c/spi/uart
filters in the first logic batch. They are stacked, not logic-input:
`lin` and `modbus` take `uart`, `avr_isp` takes `spi`, `i2cfilter` and
`i2cdemux` take `i2c`. There is no `manchester`, `rs485`, `spifilter` or
`uartfilter` decoder in 0.5.3. The order below keeps the intent (the
buses people use first, then stacked, then the rest) and respects the
dependency graph.

| Batch | Scope | Decoders | Depends on |
|---|---|---|---|
| Done | Pilot: the three existing native decoders | `i2c`, `spi`, `uart` | runtime |
| 1 | Logic front-ends for buses people use | `can`, `jtag`, `swd`, `usb_signalling`, `usb_power_delivery`, `onewire_link`, `ir_nec`, `mdio` | — |
| 2 | Logic retro/parallel/PC buses | `gpib`, `microwire`, `parallel`, `ieee488`, `ps2`, `lpc`, `z80`, `mcs48` | — |
| 3 | Logic timing, encoding and utilities | `counter`, `guess_bitrate`, `jitter`, `pwm`, `timing`, `graycode`, `morse`, `miller`, `rc_encode` | — |
| 4 | Logic audio, display, lighting and IR | `ir_rc5`, `ir_rc6`, `cec`, `dali`, `dmx512`, `dcf77`, `spdif`, `i2s`, `tdm_audio` | — |
| 5 | Logic storage, LEDs, RFID and displays | `sdcard_sd`, `st7735`, `rgb_led_ws281x`, `sda2506`, `seven_segment`, `stepper_motor`, `wiegand`, `em4100`, `em4305` | — |
| 6 | Logic IC, sensor and debug decoders | `t55xx`, `am230x`, `dsi`, `qi`, `flexray`, `avr_pdi`, `swim`, `ac97`, `aud` | — |
| 7 | Remaining logic decoders | `iec`, `maple_bus`, `ook`, `signature`, `tlc5620` | L1 (ook) |
| 8 | Stacked on i2c: filters and common devices | `i2cfilter`, `i2cdemux`, `eeprom24xx`, `lm75`, `ds1307`, `rtc8564`, `pca9571`, `tca6408a` | pilot |
| 9 | Stacked on i2c: the rest | `atsha204a`, `edid`, `hdcp`, `mlx90614`, `mxc6225xu`, `nunchuk`, `xfp` | pilot |
| 10 | Stacked on spi: flash, ADCs and radios | `avr_isp`, `spiflash`, `sdcard_spi`, `ade77xx`, `adf435x`, `adns5020`, `cc1101`, `enc28j60` | pilot |
| 11 | Stacked on spi: the rest | `max7219`, `mrf24j40`, `nes_gamepad`, `nrf24l01`, `rfm12`, `rgb_led_spi`, `ssi32`, `x2444m` | pilot |
| 12 | Stacked on uart: LIN, Modbus, MIDI and traces | `lin`, `modbus`, `midi`, `pan1321`, `amulet_ascii`, `arm_etmv3`, `arm_itm`, `arm_tpiu` | pilot |
| 13 | Stacked on L1/L7 outputs, the tail | `onewire_network`, `ds2408`, `ds243x`, `ds28ea00`, `eeprom93xx`, `jtag_ejtag`, `jtag_stm32`, `cfp`, `usb_packet`, `usb_request`, `ook_oregon`, `ook_vis` | L1, L2, L7 |

Batches 8, 10 and 12 only need the already-native `i2c`, `spi`
and `uart` bases, so they can be pulled forward if the stacked work is
more useful earlier. Batch 13 must wait for `jtag`, `onewire_link`,
`usb_signalling`, `mdio`, `microwire` and `ook`.

## Checklist

Status is `native` + `parity-tested`, `pending`, or `exception`.

| Decoder | Kind | Batch | Status | Notes |
|---|---|---|---|---|
| `ac97` | logic | 6 | pending | — |
| `ade77xx` | stacked | 3 | pending | — |
| `adf435x` | stacked | 3 | pending | — |
| `adns5020` | stacked | 3 | pending | — |
| `am230x` | logic | 6 | native, parity-tested | — |
| `amulet_ascii` | stacked | 5 | pending | — |
| `arm_etmv3` | stacked | 5 | pending | — |
| `arm_itm` | stacked | 5 | pending | — |
| `arm_tpiu` | stacked | 5 | pending | — |
| `atsha204a` | stacked | 2 | pending | — |
| `aud` | logic | 6 | pending | — |
| `avr_isp` | stacked | 3 | native, parity-tested | stacked on spi, not logic as the suggested order had it |
| `avr_pdi` | logic | 6 | pending | — |
| `can` | logic | 1 | native, parity-tested | — |
| `cc1101` | stacked | 3 | pending | — |
| `cec` | logic | 4 | native, parity-tested | — |
| `cfp` | stacked | 6 | pending | — |
| `counter` | logic | 3 | native, parity-tested | — |
| `dali` | logic | 4 | native, parity-tested | — |
| `dcf77` | logic | 4 | native, parity-tested | — |
| `dmx512` | logic | 4 | native, parity-tested | test.conf stacks uart→dmx512 (no annotations either side); the fixture tests logic input |
| `ds1307` | stacked | 1 | pending | — |
| `ds2408` | stacked | 6 | pending | — |
| `ds243x` | stacked | 6 | pending | — |
| `ds28ea00` | stacked | 6 | pending | — |
| `dsi` | logic | 6 | native, parity-tested | — |
| `edid` | stacked | 2 | pending | — |
| `eeprom24xx` | stacked | 1 | native, parity-tested | — |
| `eeprom93xx` | stacked | 6 | pending | — |
| `em4100` | logic | 5 | native, parity-tested | — |
| `em4305` | logic | 5 | native, parity-tested | — |
| `enc28j60` | stacked | 3 | pending | — |
| `flexray` | logic | 6 | native, parity-tested | — |
| `gpib` | logic | 2 | native, parity-tested | no sigrok-test dir; covered through ieee488 captures |
| `graycode` | logic | 3 | native, parity-tested | — |
| `guess_bitrate` | logic | 3 | native, parity-tested | — |
| `hdcp` | stacked | 2 | pending | — |
| `i2c` | logic | pilot | native, parity-tested | pilot |
| `i2cdemux` | stacked | 1 | native, parity-tested | stacked on i2c |
| `i2cfilter` | stacked | 1 | native, parity-tested | stacked on i2c |
| `i2s` | logic | 4 | native, parity-tested | — |
| `iec` | logic | 7 | native, parity-tested | — |
| `ieee488` | logic | 2 | native, parity-tested | — |
| `ir_nec` | logic | 1 | native, parity-tested | — |
| `ir_rc5` | logic | 4 | native, parity-tested | — |
| `ir_rc6` | logic | 4 | native, parity-tested | — |
| `jitter` | logic | 3 | native, parity-tested | — |
| `jtag` | logic | 1 | native, parity-tested | — |
| `jtag_ejtag` | stacked | 6 | pending | — |
| `jtag_stm32` | stacked | 6 | pending | — |
| `lin` | stacked | 5 | native, parity-tested | stacked on uart, not logic as the suggested order had it |
| `lm75` | stacked | 1 | pending | — |
| `lpc` | logic | 2 | native, parity-tested | — |
| `maple_bus` | logic | 7 | native, parity-tested | — |
| `max7219` | stacked | 4 | pending | — |
| `mcs48` | logic | 2 | native, parity-tested | — |
| `mdio` | logic | 1 | native, parity-tested | — |
| `microwire` | logic | 2 | native, parity-tested | — |
| `midi` | stacked | 5 | native, parity-tested | — |
| `miller` | logic | 3 | native, parity-tested | — |
| `mlx90614` | stacked | 2 | pending | — |
| `modbus` | stacked | 5 | native, parity-tested | stacked on uart, not logic as the suggested order had it |
| `morse` | logic | 3 | native, parity-tested | no test.conf; synthetic VCD fixture |
| `mrf24j40` | stacked | 4 | pending | — |
| `mxc6225xu` | stacked | 2 | pending | — |
| `nes_gamepad` | stacked | 4 | pending | — |
| `nrf24l01` | stacked | 4 | pending | — |
| `nunchuk` | stacked | 2 | pending | — |
| `onewire_link` | logic | 1 | native, parity-tested | — |
| `onewire_network` | stacked | 6 | pending | — |
| `ook` | logic | 7 | native, parity-tested | — |
| `ook_oregon` | stacked | 6 | pending | — |
| `ook_vis` | stacked | 6 | pending | — |
| `pan1321` | stacked | 5 | pending | — |
| `parallel` | logic | 2 | native, parity-tested | — |
| `pca9571` | stacked | 1 | pending | — |
| `ps2` | logic | 2 | native, parity-tested | — |
| `pwm` | logic | 3 | native, parity-tested | — |
| `qi` | logic | 6 | native, parity-tested | — |
| `rc_encode` | logic | 3 | native, parity-tested | — |
| `rfm12` | stacked | 4 | pending | — |
| `rgb_led_spi` | stacked | 4 | pending | — |
| `rgb_led_ws281x` | logic | 5 | native, parity-tested | — |
| `rtc8564` | stacked | 1 | pending | — |
| `sda2506` | logic | 5 | native, parity-tested | — |
| `sdcard_sd` | logic | 5 | native, parity-tested | — |
| `sdcard_spi` | stacked | 3 | pending | — |
| `seven_segment` | logic | 5 | native, parity-tested | — |
| `signature` | logic | 7 | native, parity-tested | — |
| `spdif` | logic | 4 | native, parity-tested | — |
| `spi` | logic | pilot | native, parity-tested | pilot |
| `spiflash` | stacked | 3 | native, parity-tested | — |
| `ssi32` | stacked | 4 | pending | — |
| `st7735` | logic | 5 | native, parity-tested | — |
| `stepper_motor` | logic | 5 | native, parity-tested | — |
| `swd` | logic | 1 | native, parity-tested | exception: `strict_start` before the first request |
| `swim` | logic | 6 | pending | — |
| `t55xx` | logic | 6 | native, parity-tested | — |
| `tca6408a` | stacked | 1 | pending | — |
| `tdm_audio` | logic | 4 | native, parity-tested | — |
| `timing` | logic | 3 | native, parity-tested | — |
| `tlc5620` | logic | 7 | native, parity-tested | — |
| `uart` | logic | pilot | native, parity-tested | pilot |
| `usb_packet` | stacked | 6 | pending | — |
| `usb_power_delivery` | logic | 1 | native, parity-tested | — |
| `usb_request` | stacked | 6 | pending | — |
| `usb_signalling` | logic | 1 | native, parity-tested | — |
| `wiegand` | logic | 5 | native, parity-tested | — |
| `x2444m` | stacked | 4 | pending | — |
| `xfp` | stacked | 2 | pending | — |
| `z80` | logic | 2 | native, parity-tested | — |

## Exceptions

One documented divergence so far:

- `swd`, option `strict_start=yes`: if a line reset arrives before the
  first request, upstream indexes `self.apdp`/`self.rw` while they are
  still `None` and raises `AttributeError`. The native decoder has no
  packet to flush at that point and continues. The corpus case with
  `strict_start=yes` that does have a request matches exactly. The
  difference is explained at the `put_python_data` call in
  `src/decoders/swd.rs`.

A decoder otherwise belongs here only when its Python behaviour cannot
be reproduced without changing the native runtime or adding a
dependency; the entry must name the exact `pd.py` lines and keep the
Python fallback. Metadata-only, stacked and error-path differences are
not exceptions.

## Appendix: full inventory

Generated from the imported `Decoder` classes. Class, row and binary
indices are the wire format; option values in parentheses are the
allowed set.

| id | kind | input | required channels | optional channels | options | annotation classes | rows | binary | meta |
|---|---|---|---|---|---|---|---|---|---|
| `ac97` | logic | logic | sync, clk | out, in, rst | — | 0 bit-out; 1 bit-in; 2 slot-out-raw; 3 slot-out-tag; 4 slot-out-cmd-addr; 5 slot-out-cmd-data; 6 slot-out-03; 7 slot-out-04; 8 slot-out-05; 9 slot-out-06; 10 slot-out-07; 11 slot-out-08; 12 slot-out-09; 13 slot-out-10; 14 slot-out-11; 15 slot-out-io-ctrl; 16 slot-in-raw; 17 slot-in-tag; 18 slot-in-sts-addr; 19 slot-in-sts-data; 20 slot-in-03; 21 slot-in-04; 22 slot-in-05; 23 slot-in-06; 24 slot-in-07; 25 slot-in-08; 26 slot-in-09; 27 slot-in-10; 28 slot-in-11; 29 slot-in-io-sts; 30 warning; 31 error | bits-out(0); slots-out-raw(2); slots-out(3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15); bits-in(1); slots-in-raw(16); slots-in(17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29); warnings(30); errors(31) | 0 frame-out; 1 frame-in; 2 slot-raw-out; 3 slot-raw-in | — |
| `ade77xx` | stacked | spi | — | — | — | 0 read; 1 write; 2 warning | read(0); write(1); warnings(2) | — | — |
| `adf435x` | stacked | spi | — | — | — | 0 register | registers(0) | — | — |
| `adns5020` | stacked | spi | — | — | — | 0 read; 1 write; 2 warning | read(0); write(1); warnings(2) | — | — |
| `am230x` | logic | logic | sda | — | device=am230x (am230x/rht, dht11) | 0 start; 1 response; 2 bit; 3 end; 4 byte; 5 humidity; 6 temperature; 7 checksum | bits(0, 1, 2, 3); bytes(4); results(5, 6, 7) | — | — |
| `amulet_ascii` | stacked | uart | — | — | ms_chan=RX (RX, TX); sm_chan=TX (RX, TX) | 0 page; 1 gbv; 2 gwv; 3 gsv; 4 glv; 5 grpc; 6 sbv; 7 swv; 8 ssv; 9 rpc; 10 line; 11 rect; 12 frect; 13 pixel; 14 gbva; 15 gwva; 16 sbva; 17 gbvr; 18 gwvr; 19 gsvr; 20 glvr; 21 grpcr; 22 sbvr; 23 swvr; 24 ssvr; 25 rpcr; 26 liner; 27 rectr; 28 frectr; 29 pixelr; 30 gbvar; 31 gwvar; 32 sbvar; 33 ack; 34 nack; 35 swva; 36 swvar; 37 gcv; 38 gcvr; 39 scv; 40 scvr; 41 bit; 42 field; 43 warning | bits(41); fields(42); commands(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40); warnings(43) | — | — |
| `arm_etmv3` | stacked | uart | — | — | objdump=arm-none-eabi-objdump; objdump_opts=-lSC; elffile=; branch_enc=alternative (alternative, original) | 0 trace; 1 branch; 2 exception; 3 execution; 4 data; 5 pc; 6 instr_e; 7 instr_n; 8 source; 9 location; 10 function | trace(0); flow(1, 2, 3); data(4); pc(5); instruction(6, 7); source(8); location(9); function(10) | — | — |
| `arm_itm` | stacked | uart | — | — | objdump=arm-none-eabi-objdump; objdump_opts=-lSC; elffile= | 0 trace; 1 timestamp; 2 software; 3 dwt_event; 4 dwt_watchpoint; 5 dwt_exc; 6 dwt_pc; 7 mode_thread; 8 mode_irq; 9 mode_exc; 10 location; 11 function | trace(0, 1); software(2); dwt_event(3); dwt_watchpoint(4); dwt_exc(5); dwt_pc(6); mode(7, 8, 9); location(10); function(11) | — | — |
| `arm_tpiu` | stacked | uart | — | — | stream=1; sync_offset=0 | 0 stream; 1 data | stream(0); data(1) | — | — |
| `atsha204a` | stacked | i2c | — | — | — | 0 waddr; 1 count; 2 opcode; 3 param1; 4 param2; 5 data; 6 crc; 7 status; 8 warning | frame(0, 1, 2, 3, 4, 5, 6); status(7); warnings(8) | — | — |
| `aud` | logic | logic | audck, naudsync, audata3, audata2, audata1, audata0 | — | — | 0 dest | — | — | — |
| `avr_isp` | stacked | spi | — | — | — | 0 pe; 1 rsb0; 2 rsb1; 3 rsb2; 4 ce; 5 rfb; 6 rhfb; 7 refb; 8 warnings; 9 dev | bits(); commands(0, 1, 2, 3, 4, 5, 6, 7); warnings(8); dev(9) | — | — |
| `avr_pdi` | logic | logic | reset, data | — | — | 0 uart-bit; 1 start-bit; 2 data-bit; 3 parity-ok; 4 parity-err; 5 stop-ok; 6 stop-err; 7 break; 8 opcode; 9 data-prog; 10 data-dev; 11 pdi-break; 12 enable; 13 disable; 14 cmd-data | uart_bits(0); uart_fields(1, 2, 3, 4, 5, 6, 7); pdi_fields(8, 9, 10, 11); pdi_cmds(12, 13, 14) | 0 bytes | — |
| `can` | logic | logic | can_rx | — | nominal_bitrate=1000000; fast_bitrate=2000000; sample_point=70.0 | 0 data; 1 sof; 2 eof; 3 id; 4 ext-id; 5 full-id; 6 ide; 7 reserved-bit; 8 rtr; 9 srr; 10 dlc; 11 crc-sequence; 12 crc-delimiter; 13 ack-slot; 14 ack-delimiter; 15 stuff-bit; 16 warnings; 17 bit | bits(15, 17); fields(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14); warnings(16) | — | — |
| `cc1101` | stacked | spi | — | — | — | 0 strobe; 1 single_read; 2 single_write; 3 burst_read; 4 burst_write; 5 status; 6 warning | cmd(0); data(1, 2, 3, 4, 5); status(6); warnings(7) | — | — |
| `cec` | logic | logic | cec | — | — | 0 st; 1 eom-0; 2 eom-1; 3 nack; 4 ack; 5 bits; 6 bytes; 7 frames; 8 sections; 9 warnings | bits(0, 1, 2, 3, 4, 5); bytes(6); frames(7); sections(8); warnings(9) | — | — |
| `cfp` | stacked | mdio | — | — | — | 0 register; 1 decode | registers(0); decodes(1) | — | — |
| `counter` | logic | logic | data | reset | data_edge=any (any, rising, falling); divider=0; reset_edge=falling (rising, falling); edge_off=0; word_off=0; dead_cycles=0; start_with_reset=no (no, yes) | 0 edge_count; 1 word_count; 2 word_reset | edge_counts(0); word_counts(1); word_resets(2) | — | — |
| `dali` | logic | logic | dali | — | polarity=active-low (active-low, active-high) | 0 bit; 1 startbit; 2 sbit; 3 ybit; 4 address; 5 command; 6 reply; 7 raw | bits(0); raw(7); fields(1, 2, 3, 4, 5, 6) | — | — |
| `dcf77` | logic | logic | data | — | — | 0 start-of-minute; 1 special-bits; 2 call-bit; 3 summer-time; 4 cest; 5 cet; 6 leap-second; 7 start-of-time; 8 minute; 9 minute-parity; 10 hour; 11 hour-parity; 12 day; 13 day-of-week; 14 month; 15 year; 16 date-parity; 17 raw-bits; 18 unknown-bits; 19 warnings | bits(17, 18); fields(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16); warnings(19) | — | — |
| `dmx512` | logic | logic | dmx | — | — | 0 bit; 1 break; 2 mab; 3 startbit; 4 stopbits; 5 startcode; 6 channel; 7 interframe; 8 interpacket; 9 data; 10 error | bits(0, 3, 4); data(9); name(1, 2, 5, 6, 7, 8); errors(10) | — | — |
| `ds1307` | stacked | i2c | — | — | — | 0 reg-seconds; 1 reg-minutes; 2 reg-hours; 3 reg-day; 4 reg-date; 5 reg-month; 6 reg-year; 7 reg-control; 8 reg-ram; 9 bit-clock-halt; 10 bit-seconds; 11 bit-reserved; 12 bit-minutes; 13 bit-12-24-hours; 14 bit-am-pm; 15 bit-hours; 16 bit-day; 17 bit-date; 18 bit-month; 19 bit-year; 20 bit-out; 21 bit-sqwe; 22 bit-rs; 23 bit-ram; 24 read-datetime; 25 write-datetime; 26 reg-read; 27 reg-write; 28 warnings | bits(9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23); regs(0, 1, 2, 3, 4, 5, 6, 7, 8); date-time(24, 25, 26, 27); warnings(28) | — | — |
| `ds2408` | stacked | onewire_network | — | — | — | 0 text | — | — | — |
| `ds243x` | stacked | onewire_network | — | — | — | 0 text | — | 0 mem_read | — |
| `ds28ea00` | stacked | onewire_network | — | — | — | 0 text | — | — | — |
| `dsi` | logic | logic | dsi | — | polarity=active-high (active-low, active-high) | 0 bit; 1 startbit; 2 level; 3 raw | bits(0); raw(3); fields(1, 2) | — | — |
| `edid` | stacked | i2c | — | — | — | 0 fields; 1 sections | fields(0); sections(1) | — | — |
| `eeprom24xx` | stacked | i2c | — | — | chip=generic (generic, microchip_24aa65, microchip_24lc65, microchip_24c65, microchip_24aa64, microchip_24lc64, microchip_24aa02uid, microchip_24aa025uid, microchip_24aa025uid_sot23, onsemi_cat24c256, onsemi_cat24m01, siemens_slx_24c01, siemens_slx_24c02, st_m24c01, st_m24c02, xicor_x24c02); addr_counter=0 | 0 warnings; 1 control-code; 2 address-pin; 3 rw-bit; 4 word-addr-byte; 5 data-byte; 6 control-word; 7 word-addr; 8 data; 9 byte-write; 10 page-write; 11 cur-addr-read; 12 random-read; 13 seq-random-read; 14 seq-cur-addr-read; 15 ack-polling; 16 set-bank-addr; 17 read-bank-addr; 18 set-wp; 19 clear-all-wp; 20 read-wp | bits-bytes(1, 2, 3, 4, 5); fields(6, 7, 8); ops(9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20); warnings(0) | 0 binary | — |
| `eeprom93xx` | stacked | microwire | — | — | addresssize=8; wordsize=16; format=hex (ascii, hex) | 0 si-data; 1 so-data; 2 warning | data(0, 1); warnings(2) | 0 address; 1 data | — |
| `em4100` | logic | logic | data | — | polarity=active-high (active-low, active-high); datarate=64 (64, 32, 16); coilfreq=125000 | 0 bit; 1 header; 2 version-customer; 3 data; 4 rowparity-ok; 5 rowparity-err; 6 colparity-ok; 7 colparity-err; 8 stopbit; 9 tag | bits(0); fields(1, 2, 3, 4, 5, 6, 7, 8); tags(9) | — | — |
| `em4305` | logic | logic | data | — | coilfreq=125000; first_field_stop=40; w_gap=12; w_one_max=32; w_zero_on_min=15; w_zero_off_max=27; em4100_decode=on (on, off) | 0 bit_value; 1 first_field_stop; 2 write_gap; 3 write_mode_exit; 4 bit; 5 opcode; 6 lock; 7 data; 8 password; 9 address; 10 bitrate | bits(0); structure(1, 2, 3, 4); fields(5, 6, 7, 8, 9); decode(10) | — | — |
| `enc28j60` | stacked | spi | — | — | — | 0 rcr; 1 rbm; 2 wcr; 3 wbm; 4 bfs; 5 bfc; 6 src; 7 data; 8 reg-addr; 9 warning | fields(7, 8); commands(0, 1, 2, 3, 4, 5, 6); warnings(9) | — | — |
| `flexray` | logic | logic | channel | — | channel_type=A (A, B); bitrate=10000000 (10000000, 5000000, 2500000) | 0 data; 1 tss; 2 fss; 3 reserved-bit; 4 ppi; 5 null-frame; 6 sync-frame; 7 startup-frame; 8 id; 9 length; 10 header-crc; 11 cycle; 12 data-byte; 13 frame-crc; 14 cid-delimiter; 15 bss; 16 warnings; 17 bit; 18 cid; 19 dts; 20 cas | bits(15, 17); fields(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 18, 19, 20); warnings(16) | — | — |
| `gpib` | logic | logic | dio1, dio2, dio3, dio4, dio5, dio6, dio7, dio8, eoi, dav, nrfd, ndac, ifc, srq, atn, ren | — | sample_total=0 | 0 items; 1 gpib; 2 eoi | bytes(0); gpib(1); eoi(2) | — | — |
| `graycode` | logic | logic | — | d0, d1, d2, d3, d4, d5, d6, d7 | edges=0; avg_period=10 | 0 phase; 1 increment; 2 count; 3 turns; 4 interval; 5 average; 6 rpm | phase(0); increment(1); count(2); turns(3); interval(4); average(5); rpm(6) | — | — |
| `guess_bitrate` | logic | logic | data | — | — | 0 bitrate | — | — | — |
| `hdcp` | stacked | i2c | — | — | — | 0 message-0x00; 1 message-0x01; 2 message-0x02; 3 message-0x03; 4 message-0x04; 5 message-0x05; 6 message-0x06; 7 message-0x07; 8 message-0x08; 9 message-0x09; 10 message-0x0A; 11 message-0x0B; 12 message-0x0C; 13 message-0x0D; 14 message-0x0E; 15 message-0x0F; 16 message-0x10; 17 message-0x11; 18 summary; 19 warnings | messages(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17); summary(18); warnings(19) | — | — |
| `i2c` | logic | logic | scl, sda | — | address_format=shifted (shifted, unshifted) | 0 start; 1 repeat-start; 2 stop; 3 ack; 4 nack; 5 bit; 6 address-read; 7 address-write; 8 data-read; 9 data-write; 10 warnings | bits(5); addr-data(0, 1, 2, 3, 4, 6, 7, 8, 9); warnings(10) | 0 address-read; 1 address-write; 2 data-read; 3 data-write | Bitrate |
| `i2cdemux` | stacked | i2c | — | — | — | — | — | — | — |
| `i2cfilter` | stacked | i2c | — | — | address=0; direction=both (read, write, both) | — | — | — | — |
| `i2s` | logic | logic | sck, ws, sd | — | — | 0 left; 1 right; 2 warnings | — | 0 wav | — |
| `iec` | logic | logic | data, clk, atn | srq | — | 0 items; 1 gpib; 2 eoi | bytes(0); gpib(1); eoi(2) | — | — |
| `ieee488` | logic | logic | dio1 | dio2, dio3, dio4, dio5, dio6, dio7, dio8, eoi, dav, nrfd, ndac, ifc, srq, atn, ren, clk | iec_periph=no (no, yes); delim=eol (none, eol) | 0 bit; 1 raw; 2 cmd; 3 laddr; 4 taddr; 5 saddr; 6 data; 7 eoi; 8 text; 9 periph; 10 warn | bits(0); raws(1); gpib(2, 3, 4, 5, 6); eois(7); texts(8); periphs(9); warns(10) | 0 raw; 1 data | — |
| `ir_nec` | logic | logic | ir | — | polarity=active-low (active-low, active-high); cd_freq=0 | 0 bit; 1 agc-pulse; 2 longpause; 3 shortpause; 4 stop-bit; 5 leader-code; 6 addr; 7 addr-inv; 8 cmd; 9 cmd-inv; 10 repeat-code; 11 remote; 12 warnings | bits(0, 1, 2, 3, 4); fields(5, 6, 7, 8, 9, 10); remote(11); warnings(12) | — | — |
| `ir_rc5` | logic | logic | ir | — | polarity=active-low (active-low, active-high); protocol=standard (standard, extended) | 0 bit; 1 startbit1; 2 startbit2; 3 togglebit-0; 4 togglebit-1; 5 address; 6 command | bits(0); fields(1, 2, 3, 4, 5, 6) | — | — |
| `ir_rc6` | logic | logic | ir | — | polarity=auto (auto, active-low, active-high) | 0 bit; 1 sync; 2 startbit; 3 field; 4 togglebit; 5 address; 6 command | bits(0); fields(1, 2, 3, 4, 5, 6) | — | — |
| `jitter` | logic | logic | clk, sig | — | clk_polarity=rising (rising, falling, both); sig_polarity=rising (rising, falling, both) | 0 jitter; 1 clk_missed; 2 sig_missed | jitter(0); clk_missed(1); sig_missed(2) | 0 ascii-float | Clock missed, Signal missed |
| `jtag` | logic | logic | tdi, tdo, tck, tms | trst, srst, rtck | — | 0 test-logic-reset; 1 run-test/idle; 2 select-dr-scan; 3 capture-dr; 4 update-dr; 5 pause-dr; 6 shift-dr; 7 exit1-dr; 8 exit2-dr; 9 select-ir-scan; 10 capture-ir; 11 update-ir; 12 pause-ir; 13 shift-ir; 14 exit1-ir; 15 exit2-ir; 16 bit-tdi; 17 bit-tdo; 18 bitstring-tdi; 19 bitstring-tdo | bits-tdi(16); bits-tdo(17); bitstrings-tdi(18); bitstrings-tdo(19); states(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15) | — | — |
| `jtag_ejtag` | stacked | jtag | — | — | — | 0 instruction; 1 reset; 2 device_id; 3 implementation; 4 data; 5 address; 6 control; 7 fastdata; 8 pc_sample; 9 bypass; 10 control_field_in; 11 control_field_out; 12 pracc | instructions(0); control_fields_in(10); control_fields_out(11); regs(1, 2, 3, 4, 5, 6, 7, 8, 9); pracc(12) | — | — |
| `jtag_stm32` | stacked | jtag | — | — | — | 0 item; 1 field; 2 command; 3 warning | items(0); fields(1); commands(2); warnings(3) | — | — |
| `lin` | stacked | uart | — | — | version=2 (1, 2) | 0 data; 1 control; 2 error; 3 inline_error | data(0, 1, 3); error(2) | — | — |
| `lm75` | stacked | i2c | — | — | sensor=lm75 (lm75); resolution=9 (9, 10, 11, 12) | 0 celsius; 1 kelvin; 2 text-verbose; 3 text; 4 warnings | — | — | — |
| `lpc` | logic | logic | lframe, lclk, lad0, lad1, lad2, lad3 | lreset, ldrq, serirq, clkrun, lpme, lpcpd, lsmi | — | 0 warnings; 1 start; 2 cycle-type; 3 addr; 4 tar1; 5 sync; 6 data; 7 tar2 | data(1, 2, 3, 4, 5, 6, 7); warnings(0) | — | — |
| `maple_bus` | logic | logic | sdcka, sdckb | — | — | 0 start; 1 end; 2 start-with-crc; 3 occupancy; 4 reset; 5 bit; 6 size; 7 source; 8 dest; 9 command; 10 data; 11 checksum; 12 frame-error; 13 checksum-error; 14 size-error | bits(0, 1, 2, 3, 4, 5); fields(6, 7, 8, 9, 10, 11); warnings(12, 13, 14) | 0 size; 1 source; 2 dest; 3 command; 4 data; 5 checksum | — |
| `max7219` | stacked | spi | — | — | — | 0 register; 1 digit; 2 warnings | commands(0, 1); warnings(2) | — | — |
| `mcs48` | logic | logic | ale, psen, d0, d1, d2, d3, d4, d5, d6, d7, a8, a9, a10, a11 | a12 | — | 0 romdata | — | 0 romdata | — |
| `mdio` | logic | logic | mdc, mdio | — | show_debug_bits=no (yes, no) | 0 bit-val; 1 bit-num; 2 frame; 3 frame-idle; 4 frame-error; 5 decode | bit-val(0); bit-num(1); frame(2, 3); frame-error(4); decode(5) | — | — |
| `microwire` | logic | logic | cs, sk, si, so | — | — | 0 start-bit; 1 si-bit; 2 so-bit; 3 status-check-ready; 4 status-check-busy; 5 warning | si-bits(0, 1); so-bits(2); status(3, 4); warnings(5) | — | — |
| `midi` | stacked | uart | — | — | — | 0 text-verbose; 1 text-sysreal-verbose; 2 text-error | normal(0, 2); sys-real(1) | — | — |
| `miller` | logic | logic | data | — | baudrate=106000; edge=falling (rising, falling, either) | 0 bit; 1 bitstring | bit(0); bitstring(1) | 0 raw | — |
| `mlx90614` | stacked | i2c | — | — | — | 0 celsius; 1 kelvin | — | — | — |
| `modbus` | stacked | uart | — | — | scchannel=RX (RX, TX); cschannel=TX (RX, TX); framegap=28 | 0 sc-server-id; 1 sc-function; 2 sc-crc; 3 sc-address; 4 sc-data; 5 sc-length; 6 sc-error; 7 cs-server-id; 8 cs-function; 9 cs-crc; 10 cs-address; 11 cs-data; 12 cs-length; 13 cs-error; 14 error-indication | sc(0, 1, 2, 3, 4, 5, 6); cs(7, 8, 9, 10, 11, 12, 13); error-indicator(14) | — | — |
| `morse` | logic | logic | data | — | timeunit=0.1 | 0 time; 1 units; 2 symbol; 3 letter; 4 word | time(0); units(1); symbol(2); letter(3); word(4) | — | — |
| `mrf24j40` | stacked | spi | — | — | — | 0 sread; 1 swrite; 2 lread; 3 lwrite; 4 warning; 5 tx-frame; 6 rx-frame; 7 tx-retry-1; 8 tx-retry-2; 9 tx-retry-3; 10 tx-fail; 11 ccafail | read(0, 2); write(1, 3); warnings(4); tx-frames(5); rx-frames(6); tx-retries-1(7); tx-retries-2(8); tx-retries-3(9); tx-fails(10); ccafails(11) | — | — |
| `mxc6225xu` | stacked | i2c | — | — | — | 0 text | — | — | — |
| `nes_gamepad` | stacked | spi | — | — | variant=Standard gamepad (Standard gamepad) | 0 button; 1 no-press; 2 not-connected | buttons(0); no-press(1); not-connected(2) | — | — |
| `nrf24l01` | stacked | spi | — | — | chip=nrf24l01 (nrf24l01, xn297) | 0 cmd; 1 tx-data; 2 register; 3 rx-data; 4 warning | commands(0, 1); responses(2, 3); warnings(4) | — | — |
| `nunchuk` | stacked | i2c | — | — | — | 0 reg-0x00; 1 reg-0x01; 2 reg-0x02; 3 reg-0x03; 4 reg-0x04; 5 reg-0x05; 6 bit-bz; 7 bit-bc; 8 bit-ax; 9 bit-ay; 10 bit-az; 11 nunchuk-write; 12 cmd-init; 13 summary; 14 warnings | regs(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12); summary(13); warnings(14) | — | — |
| `onewire_link` | logic | logic | owr | — | overdrive=no (yes, no) | 0 bit; 1 warnings; 2 reset; 3 presence; 4 overdrive | bits(0, 2, 3); info(4); warnings(1) | — | — |
| `onewire_network` | stacked | onewire_link | — | — | — | 0 text | — | — | — |
| `ook` | logic | logic | data | — | invert=no (no, yes); decodeas=Manchester (NRZ, Manchester, Diff Manchester); preamble=auto (auto, 1010, 1111); preamlen=7 (0, 3, 4, 5, 6, 7, 8, 9, 10); diffmanvar=1 (1, 0) | 0 frame; 1 info; 2 1111; 3 1010; 4 diffman; 5 nrz | frame(0); info(1); man1111(2); man1010(3); diffman(4); nrz(5) | 0 pulse-lengths | — |
| `ook_oregon` | stacked | ook | — | — | unknown=Unknown (Unknown, Temp, Temp_Hum, Temp_Hum1, Temp_Hum_Baro, Temp_Hum_Baro1, UV, UV1, Wind, Rain, Rain1) | 0 bit; 1 field; 2 l2; 3 pre; 4 syn; 5 id; 6 ch; 7 roll; 8 f1 | bits(0); fields(1, 3, 4); l2(2) | 0 data-hex | — |
| `ook_vis` | stacked | ook | — | — | displayas=Nibble - Hex (Byte - Hex, Byte - Hex rev, Byte - BCD, Byte - BCD rev, Nibble - Hex, Nibble - Hex rev, Nibble - BCD, Nibble - BCD rev); synclen=4 (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10); syncoffset=0 (-4, -3, -2, -1, 0, 1, 2, 3, 4); refsample=off (off, show numbers, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30) | 0 bit; 1 ref; 2 field; 3 ref_field; 4 level2; 5 ref_level2 | bits(0); compare(1); fields(2); ref_fields(3); level2(4); ref_level2(5) | — | — |
| `pan1321` | stacked | uart | — | — | — | 0 text-verbose; 1 text; 2 warnings | — | — | — |
| `parallel` | logic | logic | — | clk, d0, d1, d2, d3, d4, d5, d6, d7 | clock_edge=rising (rising, falling); wordsize=0; endianness=little (little, big) | 0 items; 1 words | items(0); words(1) | — | — |
| `pca9571` | stacked | i2c | — | — | — | 0 register; 1 value; 2 warning | regs(0, 1); warnings(2) | — | — |
| `ps2` | logic | logic | clk, data | — | — | 0 bit; 1 start-bit; 2 stop-bit; 3 parity-ok; 4 parity-err; 5 data-bit; 6 word | bits(0); fields(1, 2, 3, 4, 5, 6) | — | — |
| `pwm` | logic | logic | data | — | polarity=active-high (active-low, active-high) | 0 duty-cycle; 1 period | duty-cycle(0); period(1) | 0 raw | Average |
| `qi` | logic | logic | qi | — | — | 0 bits; 1 bytes-errors; 2 bytes-start; 3 bytes-info; 4 bytes-data; 5 packets-data; 6 packets-checksum-ok; 7 packets-checksum-err | bits(0); bytes(1, 2, 3, 4); packets(5, 6, 7) | — | — |
| `rc_encode` | logic | logic | data | — | remote=none (none, maplin_l95ar) | 0 bit-0; 1 bit-1; 2 bit-f; 3 bit-U; 4 bit-sync; 5 pin; 6 code-word-addr; 7 code-word-data | bits(0, 1, 2, 3, 4); pins(5); code-words(6, 7) | — | — |
| `rfm12` | stacked | spi | — | — | — | 0 cmd; 1 params; 2 disabled; 3 return; 4 disabled_return; 5 interpretation | commands(0, 1, 2); return(3, 4); interpretation(5) | — | — |
| `rgb_led_spi` | stacked | spi | — | — | — | 0 rgb | — | — | — |
| `rgb_led_ws281x` | logic | logic | din | — | — | 0 bit; 1 reset; 2 rgb | bit(0, 1); rgb(2) | — | — |
| `rtc8564` | stacked | i2c | — | — | — | 0 reg-0x00; 1 reg-0x01; 2 reg-0x02; 3 reg-0x03; 4 reg-0x04; 5 reg-0x05; 6 reg-0x06; 7 reg-0x07; 8 reg-0x08; 9 read; 10 write; 11 bit-reserved; 12 bit-vl; 13 bit-century; 14 reg-read; 15 reg-write | bits(0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13); regs(14, 15); date-time(9, 10) | — | — |
| `sda2506` | logic | logic | clk, d, ce | — | — | 0 cmdbit; 1 databit; 2 cmd; 3 data; 4 warnings | bits(0, 1); data(3); commands(2); warnings(4) | — | — |
| `sdcard_sd` | logic | logic | cmd, clk | dat0, dat1, dat2, dat3 | — | 0 cmd0; 1 cmd1; 2 cmd2; 3 cmd3; 4 cmd4; 5 cmd5; 6 cmd6; 7 cmd7; 8 cmd8; 9 cmd9; 10 cmd10; 11 cmd11; 12 cmd12; 13 cmd13; 14 cmd14; 15 cmd15; 16 cmd16; 17 cmd17; 18 cmd18; 19 cmd19; 20 cmd20; 21 cmd21; 22 cmd22; 23 cmd23; 24 cmd24; 25 cmd25; 26 cmd26; 27 cmd27; 28 cmd28; 29 cmd29; 30 cmd30; 31 cmd31; 32 cmd32; 33 cmd33; 34 cmd34; 35 cmd35; 36 cmd36; 37 cmd37; 38 cmd38; 39 cmd39; 40 cmd40; 41 cmd41; 42 cmd42; 43 cmd43; 44 cmd44; 45 cmd45; 46 cmd46; 47 cmd47; 48 cmd48; 49 cmd49; 50 cmd50; 51 cmd51; 52 cmd52; 53 cmd53; 54 cmd54; 55 cmd55; 56 cmd56; 57 cmd57; 58 cmd58; 59 cmd59; 60 cmd60; 61 cmd61; 62 cmd62; 63 cmd63; 64 acmd0; 65 acmd1; 66 acmd2; 67 acmd3; 68 acmd4; 69 acmd5; 70 acmd6; 71 acmd7; 72 acmd8; 73 acmd9; 74 acmd10; 75 acmd11; 76 acmd12; 77 acmd13; 78 acmd14; 79 acmd15; 80 acmd16; 81 acmd17; 82 acmd18; 83 acmd19; 84 acmd20; 85 acmd21; 86 acmd22; 87 acmd23; 88 acmd24; 89 acmd25; 90 acmd26; 91 acmd27; 92 acmd28; 93 acmd29; 94 acmd30; 95 acmd31; 96 acmd32; 97 acmd33; 98 acmd34; 99 acmd35; 100 acmd36; 101 acmd37; 102 acmd38; 103 acmd39; 104 acmd40; 105 acmd41; 106 acmd42; 107 acmd43; 108 acmd44; 109 acmd45; 110 acmd46; 111 acmd47; 112 acmd48; 113 acmd49; 114 acmd50; 115 acmd51; 116 acmd52; 117 acmd53; 118 acmd54; 119 acmd55; 120 acmd56; 121 acmd57; 122 acmd58; 123 acmd59; 124 acmd60; 125 acmd61; 126 acmd62; 127 acmd63; 128 bits; 129 field-start; 130 field-transmission; 131 field-cmd; 132 field-arg; 133 field-crc; 134 field-end; 135 decoded-bits; 136 decoded-fields | raw-bits(128); decoded-bits(135); decoded-fields(136); fields(129, 130, 131, 132, 133, 134); cmd(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127) | — | — |
| `sdcard_spi` | stacked | spi | — | — | — | 0 cmd0; 1 cmd1; 2 cmd2; 3 cmd3; 4 cmd4; 5 cmd5; 6 cmd6; 7 cmd7; 8 cmd8; 9 cmd9; 10 cmd10; 11 cmd11; 12 cmd12; 13 cmd13; 14 cmd14; 15 cmd15; 16 cmd16; 17 cmd17; 18 cmd18; 19 cmd19; 20 cmd20; 21 cmd21; 22 cmd22; 23 cmd23; 24 cmd24; 25 cmd25; 26 cmd26; 27 cmd27; 28 cmd28; 29 cmd29; 30 cmd30; 31 cmd31; 32 cmd32; 33 cmd33; 34 cmd34; 35 cmd35; 36 cmd36; 37 cmd37; 38 cmd38; 39 cmd39; 40 cmd40; 41 cmd41; 42 cmd42; 43 cmd43; 44 cmd44; 45 cmd45; 46 cmd46; 47 cmd47; 48 cmd48; 49 cmd49; 50 cmd50; 51 cmd51; 52 cmd52; 53 cmd53; 54 cmd54; 55 cmd55; 56 cmd56; 57 cmd57; 58 cmd58; 59 cmd59; 60 cmd60; 61 cmd61; 62 cmd62; 63 cmd63; 64 acmd0; 65 acmd1; 66 acmd2; 67 acmd3; 68 acmd4; 69 acmd5; 70 acmd6; 71 acmd7; 72 acmd8; 73 acmd9; 74 acmd10; 75 acmd11; 76 acmd12; 77 acmd13; 78 acmd14; 79 acmd15; 80 acmd16; 81 acmd17; 82 acmd18; 83 acmd19; 84 acmd20; 85 acmd21; 86 acmd22; 87 acmd23; 88 acmd24; 89 acmd25; 90 acmd26; 91 acmd27; 92 acmd28; 93 acmd29; 94 acmd30; 95 acmd31; 96 acmd32; 97 acmd33; 98 acmd34; 99 acmd35; 100 acmd36; 101 acmd37; 102 acmd38; 103 acmd39; 104 acmd40; 105 acmd41; 106 acmd42; 107 acmd43; 108 acmd44; 109 acmd45; 110 acmd46; 111 acmd47; 112 acmd48; 113 acmd49; 114 acmd50; 115 acmd51; 116 acmd52; 117 acmd53; 118 acmd54; 119 acmd55; 120 acmd56; 121 acmd57; 122 acmd58; 123 acmd59; 124 acmd60; 125 acmd61; 126 acmd62; 127 acmd63; 128 r1; 129 r1b; 130 r2; 131 r3; 132 r7; 133 bits; 134 bit-warnings | bits(133, 134); cmd-reply(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132) | — | — |
| `seven_segment` | logic | logic | a, b, c, d, e, f, g | dp | polarity=common-cathode (common-cathode, common-anode) | 0 decoded-digit | decoded-digits(0) | — | — |
| `signature` | logic | logic | start, stop, clk, data | — | start_edge=rising (rising, falling); stop_edge=rising (rising, falling); clk_edge=falling (rising, falling); annbits=no (yes, no) | 0 bit0; 1 bit1; 2 start; 3 stop; 4 sig | bits(0, 1, 2, 3); sig(4) | — | — |
| `spdif` | logic | logic | data | — | — | 0 bitrate; 1 preamble; 2 bits; 3 aux; 4 samples; 5 validity; 6 subcode; 7 chan_stat; 8 parity | bits(2); info(0, 1, 3, 5, 6, 7, 8); samples(4) | — | — |
| `spi` | logic | logic | clk | miso, mosi, cs | cs_polarity=active-low (active-low, active-high); cpol=0 (0, 1); cpha=0 (0, 1); bitorder=msb-first (msb-first, lsb-first); wordsize=8 | 0 miso-data; 1 mosi-data; 2 miso-bits; 3 mosi-bits; 4 warnings; 5 miso-transfer; 6 mosi-transfer | miso-bits(2); miso-data(0); miso-transfer(5); mosi-bits(3); mosi-data(1); mosi-transfer(6); other(4) | 0 miso; 1 mosi | Bitrate |
| `spiflash` | stacked | spi | — | — | chip=adesto_at45db161e (adesto_at45db161e, atmel_at25128, atmel_at25256, fidelix_fm25q32, macronix_mx25l1605d, macronix_mx25l3205d, macronix_mx25l6405d, winbond_w25q80dv); format=hex (hex, ascii) | 0 wrsr; 1 pp; 2 read; 3 wrdi; 4 rdsr; 5 wren; 6 fast/read; 7 se; 8 rdscur; 9 wrscur; 10 rdsr2; 11 ce; 12 esry; 13 dsry; 14 write1; 15 write2; 16 rems; 17 rdid; 18 rdp/res; 19 cp; 20 enso; 21 dp; 22 2read; 23 exso; 24 ce2; 25 status; 26 be; 27 rems2; 28 bit; 29 field; 30 warning | bits(28); fields(29); commands(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27); warnings(30) | — | — |
| `ssi32` | stacked | spi | — | — | msgsize=64 | 0 ctrl-tx; 1 ack-tx; 2 ctrl-rx; 3 ack-rx | tx(0, 1); rx(2, 3) | — | — |
| `st7735` | logic | logic | cs, clk, mosi, dc | — | — | 0 bit; 1 command; 2 data; 3 description | bits(0); fields(1, 2); description(3) | — | — |
| `stepper_motor` | logic | logic | step, dir | — | unit=steps (steps, mm); steps_per_mm=100.0 | 0 speed; 1 position | speed(0); position(1) | — | — |
| `swd` | logic | logic | swclk, swdio | — | strict_start=no (yes, no) | 0 reset; 1 enable; 2 read; 3 write; 4 ack; 5 data; 6 parity | — | — | — |
| `swim` | logic | logic | swim | — | debug=no (yes, no) | 0 bit; 1 enterseq; 2 start-host; 3 start-target; 4 parity; 5 ack; 6 nack; 7 byte-write; 8 byte-read; 9 cmd-unknown; 10 cmd; 11 bytes; 12 address; 13 data-write; 14 data-read; 15 debug | bits(0); framing(2, 3, 4, 5, 6, 7, 8); protocol(1, 9, 10, 11, 12, 13, 14); debug(15) | 0 tx; 1 rx | — |
| `t55xx` | logic | logic | data | — | coilfreq=125000; start_gap=20; w_gap=20; w_one_min=48; w_one_max=63; w_zero_min=16; w_zero_max=31; em4100_decode=on (on, off) | 0 bit_value; 1 start_gap; 2 write_gap; 3 write_mode_exit; 4 bit; 5 opcode; 6 lock; 7 data; 8 password; 9 address; 10 bitrate | bits(0); structure(1, 2, 3, 4); fields(5, 6, 7, 8, 9); decode(10) | — | — |
| `tca6408a` | stacked | i2c | — | — | — | 0 register; 1 value; 2 warnings | regs(0, 1); warnings(2) | — | — |
| `tdm_audio` | logic | logic | clock, frame, data | — | bps=16; channels=8; edge=rising (rising, falling) | 0 ch0; 1 ch1; 2 ch2; 3 ch3; 4 ch4; 5 ch5; 6 ch6; 7 ch7 | ch0(0); ch1(1); ch2(2); ch3(3); ch4(4); ch5(5); ch6(6); ch7(7) | — | — |
| `timing` | logic | logic | data | — | avg_period=100; edge=any (any, rising, falling); delta=no (yes, no) | 0 time; 1 average; 2 delta | time(0); average(1); delta(2) | — | — |
| `tlc5620` | logic | logic | clk, data | load, ldac | vref_a=3.3; vref_b=3.3; vref_c=3.3; vref_d=3.3 | 0 dac-select; 1 gain; 2 value; 3 data-latch; 4 ldac-fall; 5 bit; 6 reg-write; 7 voltage-update; 8 voltage-update-all; 9 invalid-cmd | bits(5); fields(0, 1, 2); registers(6, 7); voltage-updates(8); events(3, 4); errors(9) | — | — |
| `uart` | logic | logic | — | rx, tx | baudrate=115200; data_bits=8 (5, 6, 7, 8, 9); parity=none (none, odd, even, zero, one, ignore); stop_bits=1.0 (0.0, 0.5, 1.0, 1.5); bit_order=lsb-first (lsb-first, msb-first); format=hex (ascii, dec, hex, oct, bin); invert_rx=no (yes, no); invert_tx=no (yes, no); rx_packet_delim=-1; tx_packet_delim=-1; rx_packet_len=-1; tx_packet_len=-1 | 0 rx-data; 1 tx-data; 2 rx-start; 3 tx-start; 4 rx-parity-ok; 5 tx-parity-ok; 6 rx-parity-err; 7 tx-parity-err; 8 rx-stop; 9 tx-stop; 10 rx-warnings; 11 tx-warnings; 12 rx-data-bits; 13 tx-data-bits; 14 rx-break; 15 tx-break; 16 rx-packet; 17 tx-packet | rx-data-bits(12); rx-data(0, 2, 4, 6, 8); rx-warnings(10); rx-break(14); rx-packets(16); tx-data-bits(13); tx-data(1, 3, 5, 7, 9); tx-warnings(11); tx-break(15); tx-packets(17) | 0 rx; 1 tx; 2 rxtx | — |
| `usb_packet` | stacked | usb_signalling | — | — | signalling=full-speed (full-speed, low-speed) | 0 sync-ok; 1 sync-err; 2 pid; 3 framenum; 4 addr; 5 ep; 6 crc5-ok; 7 crc5-err; 8 data; 9 crc16-ok; 10 crc16-err; 11 packet-out; 12 packet-in; 13 packet-sof; 14 packet-setup; 15 packet-data0; 16 packet-data1; 17 packet-data2; 18 packet-mdata; 19 packet-ack; 20 packet-nak; 21 packet-stall; 22 packet-nyet; 23 packet-pre; 24 packet-err; 25 packet-split; 26 packet-ping; 27 packet-reserved; 28 packet-invalid | fields(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10); packet(11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28) | — | — |
| `usb_power_delivery` | logic | logic | cc1 | cc2 | fulltext=no (yes, no) | 0 type; 1 preamble; 2 sop; 3 header; 4 data; 5 crc; 6 eop; 7 sym; 8 warnings; 9 src; 10 snk; 11 payload; 12 text | 4b5b(7); phase(1, 2, 3, 4, 5, 6); payload(11); type(0, 9, 10); warnings(8); text(12) | 0 raw-data | Bitrate |
| `usb_request` | stacked | usb_packet | — | — | in_request_start=submit (submit, first-ack) | 0 request-setup-read; 1 request-setup-write; 2 request-bulk-read; 3 request-bulk-write; 4 errors | request-setup(0, 1); request-in(2); request-out(3); errors(4) | 0 pcap | — |
| `usb_signalling` | logic | logic | dp, dm | — | signalling=automatic (automatic, full-speed, low-speed) | 0 sym-j; 1 sym-k; 2 sym-se0; 3 sym-se1; 4 sop; 5 eop; 6 bit; 7 stuffbit; 8 error; 9 keep-alive; 10 reset | bits(4, 5, 6, 7, 8, 9, 10); symbols(0, 1, 2, 3) | — | — |
| `wiegand` | logic | logic | d0, d1 | — | active=low (low, high); bitwidth_ms=4 (1, 2, 4, 8, 16, 32) | 0 bits; 1 state | bits(0); state(1) | — | — |
| `x2444m` | stacked | spi | — | — | — | 0 wrds; 1 sto; 2 sleep; 3 write; 4 wren; 5 rcl; 6 read; 7 read | — | — | — |
| `xfp` | stacked | i2c | — | — | — | 0 fieldnames-and-values; 1 fields | — | — | — |
| `z80` | logic | logic | d0, d1, d2, d3, d4, d5, d6, d7, m1, rd, wr | mreq, iorq, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15 | — | 0 addr; 1 memrd; 2 memwr; 3 iord; 4 iowr; 5 instr; 6 rop; 7 wop; 8 warn | addrbus(0); databus(1, 2, 3, 4); instructions(5); operands(6, 7); warnings(8) | — | — |
