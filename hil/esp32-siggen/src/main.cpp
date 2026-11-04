// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

// LogicPort test signal generator for ESP32 DevKit V1 (Arduino framework).
// Serial (921600 baud) line commands:
//   PING                      -> PONG
//   MAP [g0 g1 ...]           -> bit i of a mask drives GPIO gi (up to 32 pins, outputs, low).
//                                No arguments: default map 25 26 27 14 (D0 D1 D2 CLK1 wiring).
//   SET <mask>                -> set outputs
//   SEQ <repeat> <mask>:<ns> [<mask>:<ns> ...]
//                             -> play steps (mask held for ns nanoseconds), whole list <repeat> times,
//                                cycle-counter timed; interrupts off if total < 20 ms. Replies DONE <cycles>
//   CLK <hz> [duty%] [gpio]   -> continuous square wave via LEDC (default GPIO14); CLK 0 stops
//   HIZ <gpio>                -> make a pin an input (release it)
//   SPI <hz> <mode> <hex>     -> hardware SPI (VSPI) transfer: SCK GPIO18, MISO GPIO19, MOSI GPIO23,
//                                CS GPIO5 (active low, driven around the transfer). Replies the MISO bytes
//   I2C <hz> <addr> <hex>     -> hardware I2C write: SDA GPIO21, SCL GPIO22 (internal pull-ups).
//                                Replies I2C <endTransmission code> (2 = address NACK)
//   ADC <gpio>                -> read a pin with the ADC, replies the level in mV
//   KICK <gpio> <ms> [ref]    -> sample a pin with the ADC continuously for ms milliseconds,
//                                alternating with pin ref (if given) to precharge the sample cap
//   EMIT <ms> <seconds> <hex> -> play a multimeter: send the hex bytes on this serial port every
//                                ms milliseconds for the given number of seconds. The setting is
//                                kept in flash, so the emission restarts after the reset that
//                                opening the port causes, and stops once the seconds are up or a
//                                line arrives. EMIT 0 0 clears it. Replies OK
// All mapped outputs of one step change with back to back register writes (W1TS/W1TC, both banks).
#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>

#define MAXPINS 32
static int map_pins[MAXPINS] = {25, 26, 27, 14};
static int map_count = 4;
static int clk_pin = 14;
static bool clk_on = false;

struct step_words { uint32_t s0, c0, s1, c1; };

static step_words words_for(uint32_t mask) {
  step_words w = {0, 0, 0, 0};
  for (int i = 0; i < map_count; i++) {
    int g = map_pins[i];
    if (clk_on && g == clk_pin) continue;
    bool on = mask & (1u << i);
    if (g < 32) { if (on) w.s0 |= 1u << g; else w.c0 |= 1u << g; }
    else { if (on) w.s1 |= 1u << (g - 32); else w.c1 |= 1u << (g - 32); }
  }
  return w;
}

static inline void IRAM_ATTR apply(const step_words &w) {
  REG_WRITE(GPIO_OUT_W1TS_REG, w.s0);
  REG_WRITE(GPIO_OUT1_W1TS_REG, w.s1);
  REG_WRITE(GPIO_OUT_W1TC_REG, w.c0);
  REG_WRITE(GPIO_OUT1_W1TC_REG, w.c1);
}

#define MAXSTEPS 512
static step_words s_words[MAXSTEPS];
static uint32_t s_cyc[MAXSTEPS];

static void IRAM_ATTR play(int n, uint32_t rep) {
  for (uint32_t r = 0; r < rep; r++)
    for (int i = 0; i < n; i++) {
      uint32_t t0 = ESP.getCycleCount();
      apply(s_words[i]);
      while ((uint32_t)(ESP.getCycleCount() - t0) < s_cyc[i]) {}
    }
}

static void set_map(const int *pins, int n) {
  map_count = n;
  for (int i = 0; i < n; i++) {
    map_pins[i] = pins[i];
    if (clk_on && pins[i] == clk_pin) continue;
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

static void stop_clk() {
  if (clk_on) {
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    pinMode(clk_pin, OUTPUT);
    digitalWrite(clk_pin, LOW);
    clk_on = false;
  }
}

static void start_clk(uint32_t hz, uint32_t duty_pct, int pin) {
  stop_clk();
  if (!hz) return;
  clk_pin = pin;
  // pick resolution so that hz * 2^bits <= 80 MHz
  int bits = 1;
  while (bits < 14 && ((uint64_t)hz << (bits + 1)) <= 80000000ULL) bits++;
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_HIGH_SPEED_MODE; t.duty_resolution = (ledc_timer_bit_t)bits;
  t.timer_num = LEDC_TIMER_0; t.freq_hz = hz; t.clk_cfg = LEDC_USE_APB_CLK;
  if (ledc_timer_config(&t) != ESP_OK) { Serial.println("ERR clk"); return; }
  ledc_channel_config_t c = {};
  c.gpio_num = pin; c.speed_mode = LEDC_HIGH_SPEED_MODE; c.channel = LEDC_CHANNEL_0;
  c.timer_sel = LEDC_TIMER_0; c.duty = ((1u << bits) * duty_pct) / 100; c.hpoint = 0;
  ledc_channel_config(&c);
  clk_on = true;
  Serial.printf("CLK %u Hz (actual %u) bits %d gpio %d\n", hz,
                ledc_get_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0), bits, pin);
}

static int parse_hex(const char *p, uint8_t *out, int max) {
  int n = 0;
  while (*p == ' ') p++;
  while (p[0] && p[1] && n < max) {
    char b[3] = {p[0], p[1], 0};
    out[n++] = strtoul(b, 0, 16);
    p += 2;
  }
  return n;
}

// Give pins back to the GPIO matrix after a peripheral used them.
static void restore_pins(const int *pins, int n) {
  for (int i = 0; i < n; i++) gpio_reset_pin((gpio_num_t)pins[i]);
  int copy[MAXPINS];
  memcpy(copy, map_pins, sizeof(copy));
  set_map(copy, map_count);
}

static void do_spi(const char *args) {
  static const int pins[4] = {18, 19, 23, 5};
  uint32_t hz = 1000000; int mode = 0, off = 0;
  sscanf(args, "%u %d %n", &hz, &mode, &off);
  uint8_t buf[256];
  int n = parse_hex(args + off, buf, sizeof(buf));
  for (int i = 0; i < 3; i++) gpio_reset_pin((gpio_num_t)pins[i]);
  // Drive CS high before enabling the output, so it doesn't glitch low.
  digitalWrite(5, HIGH);
  pinMode(5, OUTPUT);
  SPI.begin(18, 19, 23, -1);
  delayMicroseconds(20);
  SPI.beginTransaction(SPISettings(hz, MSBFIRST, mode));
  digitalWrite(5, LOW);
  SPI.transfer(buf, n);
  digitalWrite(5, HIGH);
  SPI.endTransaction();
  SPI.end();
  restore_pins(pins, 4);
  Serial.print("SPI");
  for (int i = 0; i < n; i++) Serial.printf(" %02X", buf[i]);
  Serial.println();
}

static void do_i2c(const char *args) {
  static const int pins[2] = {21, 22};
  uint32_t hz = 100000; int addr = 0, off = 0;
  sscanf(args, "%u %i %n", &hz, &addr, &off);
  uint8_t buf[128];
  int n = parse_hex(args + off, buf, sizeof(buf));
  for (int i = 0; i < 2; i++) gpio_reset_pin((gpio_num_t)pins[i]);
  Wire.begin(21, 22, hz);
  Wire.beginTransmission(addr);
  Wire.write(buf, n);
  int r = Wire.endTransmission();
  Wire.end();
  restore_pins(pins, 2);
  Serial.printf("I2C %d\n", r);
}

static Preferences prefs;

// Plays the stored multimeter packet until its time is up or a byte arrives.
static void emit_stored() {
  prefs.begin("siggen", false);
  uint32_t ms = prefs.getUInt("emit_ms", 0), secs = prefs.getUInt("emit_s", 0);
  uint8_t pkt[256]; size_t n = prefs.getBytes("emit_pkt", pkt, sizeof(pkt));
  if (!ms || !secs || !n) { prefs.end(); return; }
  uint32_t t0 = millis();
  while (millis() - t0 < secs * 1000UL && !Serial.available()) {
    Serial.write(pkt, n); Serial.flush();
    delay(ms);
  }
  prefs.clear();
  prefs.end();
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.begin(921600);
  static const int def[4] = {25, 26, 27, 14};
  set_map(def, 4);
  emit_stored();
  Serial.println("SIGGEN READY");
}

static void handle(String &line) {
  if (line == "PING") Serial.println("PONG");
  else if (line == "MAP" || line.startsWith("MAP ")) {
    int pins[MAXPINS], n = 0;
    const char *p = line.c_str() + 3; char *e;
    while (n < MAXPINS) {
      while (*p == ' ') p++;
      if (!*p) break;
      long g = strtol(p, &e, 0);
      if (e == p || g < 0 || g > 33) { Serial.println("ERR pin"); return; }
      pins[n++] = g; p = e;
    }
    if (!n) { static const int def[4] = {25, 26, 27, 14}; set_map(def, 4); }
    else set_map(pins, n);
    Serial.printf("OK %d\n", map_count);
  } else if (line.startsWith("SET ")) {
    apply(words_for(strtoul(line.c_str() + 4, 0, 0))); Serial.println("OK");
  } else if (line.startsWith("HIZ ")) {
    pinMode(atoi(line.c_str() + 4), INPUT); Serial.println("OK");
  } else if (line.startsWith("SPI ")) {
    do_spi(line.c_str() + 4);
  } else if (line.startsWith("I2C ")) {
    do_i2c(line.c_str() + 4);
  } else if (line.startsWith("ADC ")) {
    Serial.printf("MV %u\n", analogReadMilliVolts(atoi(line.c_str() + 4)));
  } else if (line.startsWith("KICK ")) {
    int pin = 0, ms = 0, ref = -1; sscanf(line.c_str() + 5, "%d %d %d", &pin, &ms, &ref);
    uint32_t t0 = millis(), n = 0;
    while (millis() - t0 < (uint32_t)ms) {
      if (ref >= 0) analogRead(ref);
      analogRead(pin); n++;
    }
    Serial.printf("DONE %u\n", n);
  } else if (line.startsWith("EMIT ")) {
    uint32_t ms = 0, secs = 0; int used = 0;
    sscanf(line.c_str() + 5, "%u %u %n", &ms, &secs, &used);
    uint8_t pkt[256]; int n = 0;
    const char *h = line.c_str() + 5 + used;
    while (h[0] && h[1] && n < (int)sizeof(pkt)) {
      char b[3] = {h[0], h[1], 0}; pkt[n++] = (uint8_t)strtoul(b, 0, 16); h += 2;
    }
    prefs.begin("siggen", false);
    prefs.clear();
    if (ms && secs && n) {
      prefs.putUInt("emit_ms", ms); prefs.putUInt("emit_s", secs); prefs.putBytes("emit_pkt", pkt, n);
    }
    prefs.end();
    Serial.println("OK");
    if (ms && secs && n) emit_stored();
  } else if (line.startsWith("CLK ")) {
    uint32_t hz = 0, duty = 50; int pin = 14;
    sscanf(line.c_str() + 4, "%u %u %d", &hz, &duty, &pin);
    if (!hz) { stop_clk(); Serial.println("OK"); } else start_clk(hz, duty, pin);
  } else if (line.startsWith("SEQ ")) {
    const char *p = line.c_str() + 4; char *e;
    uint32_t rep = strtoul(p, &e, 0); p = e; int n = 0; uint64_t total = 0;
    while (n < MAXSTEPS) {
      while (*p == ' ') p++;
      if (!*p) break;
      uint32_t m = strtoul(p, &e, 0); if (*e != ':') break; p = e + 1;
      uint64_t ns = strtoull(p, &e, 0); p = e;
      s_words[n] = words_for(m); s_cyc[n] = (uint32_t)((ns * 240ULL) / 1000ULL); total += s_cyc[n]; n++;
    }
    total *= rep;
    uint32_t t0 = ESP.getCycleCount();
    if (total < 240ULL * 20000ULL) { portDISABLE_INTERRUPTS(); play(n, rep); portENABLE_INTERRUPTS(); }
    else play(n, rep);
    Serial.printf("DONE %u steps x%u cycles=%u\n", n, rep, (uint32_t)(ESP.getCycleCount() - t0));
  } else Serial.println("ERR");
}

void loop() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { line += ch; continue; }
    line.trim();
    if (line.length()) handle(line);
    line = "";
  }
}
