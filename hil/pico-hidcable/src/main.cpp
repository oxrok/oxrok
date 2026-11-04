// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

// HID cable emulator: the Pico enumerates as one of the USB HID adapters
// that handheld multimeters come with, and plays a meter packet.
//
//   bu86x   Brymen BU-86X (0820:0001): 8 byte reports, the host asks with
//           00 xx 66 and gets 24 bytes in three reports
//   ch9325  WCH CH9325 (1a86:e008): 7 data bytes per report behind a
//           0xf0|count byte, streamed every PERIOD ms, line parameters
//           arrive as a feature report
//   victor  Victor 70C/86C cable (1244:d237): 14 byte reports, scrambled
//   usbrelay  dcttech USBRelay2 (16c0:05df): feature report with the serial
//           number ABCDE and the relay state, switched by feature reports
//   pickit2  Microchip PICkit2 (04d8:0033): 64 byte reports; SETUP answers
//           a status after 300 ms, SETPOS and READ return a counter pattern
//           (pin4 toggles every sample, pin5 every two, pin6 every four)
//   usbtmc  a Rigol DS1102E (1ab1:0588) on USBTMC: the DS1000 SCPI subset
//           the sigrok driver uses, 600 point live waveforms (a sine on
//           CH1, a ramp on CH2) and the 1 M point memory readout
//
// Commands on the USB serial port (115200, any speed): MODE name,
// PACKET hex, PERIOD ms, STATUS, SAVE, REBOOT. MODE and SAVE write the
// settings to flash; a mode change reboots so the new descriptor shows.

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <EEPROM.h>
#include <math.h>

enum Mode : uint8_t { BU86X = 0, CH9325 = 1, VICTOR = 2, USBRELAY = 3, PICKIT2 = 4, USBTMC = 5 };

struct Settings {
  uint32_t magic;
  uint8_t mode;
  uint8_t packet_len;
  uint16_t period_ms;
  uint8_t packet[64];
};

static const uint32_t MAGIC = 0x48494431; // "HID1"
static Settings cfg;
static Adafruit_USBD_HID usb_hid;
static unsigned long last_emit = 0;
static uint8_t last_feature[16];
static size_t last_feature_len = 0;
// Set by the USB callback, served from loop(): the callback runs in
// interrupt context where reports cannot be sent.
static volatile bool emit_requested = false;

// A BM867s packet: 1.2345 V AC with 50.0 Hz on the second display.
static const uint8_t default_bm86x[24] = {
  0x00, 0x00, 0x01, 0xa0, 0xdb, 0xf8, 0xe4, 0x7c, 0x01, 0x00, 0x7c, 0xbe,
  0xbf, 0x00, 0x04, 0x00, 0x86, 0x86, 0x86, 0x86, 0x00, 0x00, 0x00, 0x00,
};

// Vendor page report descriptor: input and output reports of IN/OUT
// bytes, a feature report of 8, no report ids.
#define CABLE_REPORT(IN, OUT)                                           \
  0x06, 0x00, 0xff, 0x09, 0x01, 0xa1, 0x01,                            \
  0x09, 0x02, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, IN,  0x81, 0x02, \
  0x09, 0x03, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, OUT, 0x91, 0x02, \
  0x09, 0x04, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x08, 0xb1, 0x02, \
  0xc0

static const uint8_t desc_8[] = { CABLE_REPORT(8, 8) };
static const uint8_t desc_14[] = { CABLE_REPORT(14, 14) };
static const uint8_t desc_64[] = { CABLE_REPORT(64, 64) };

// usbrelay state.
static uint8_t relay_state = 0;

// pickit2 state: the raw sample dump as the device stores it, two
// samples per byte, newest first.
static uint8_t pk_raw[512];
static uint8_t pk_bank = 0, pk_offset = 0, pk_reads = 0;
static volatile bool pk_status_pending = false, pk_read_pending = false;
static unsigned long pk_status_due = 0;

static uint8_t pk_nibble(unsigned k) {
  uint8_t n = 0;
  if (k & 1) n |= 0x04; // pin4
  if (k & 2) n |= 0x08; // pin5
  if (k & 4) n |= 0x01; // pin6
  return n;
}

// USBTMC: the interface descriptor and a small Rigol DS1000 emulation.
class UsbtmcInterface : public Adafruit_USBD_Interface {
public:
  uint16_t getInterfaceDescriptor(uint8_t, uint8_t *buf, uint16_t bufsize) override {
    uint8_t itf = 0, ep_in = 0, ep_out = 0;
    if (buf) {
      itf = TinyUSBDevice.allocInterface(1);
      ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
      ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    }
    uint8_t const desc[] = {
      TUD_USBTMC_IF_DESCRIPTOR(itf, 2, _strid, TUD_USBTMC_PROTOCOL_USB488),
      TUD_USBTMC_BULK_DESCRIPTORS(ep_out, ep_in, 64),
    };
    uint16_t len = sizeof(desc);
    if (buf) {
      if (bufsize < len) return 0;
      memcpy(buf, desc, len);
    }
    return len;
  }
};
static UsbtmcInterface usbtmc_itf;

static char tmc_cmd[128];
static size_t tmc_cmd_len = 0;
// The response: a text or a waveform header, then generated data bytes.
static uint8_t tmc_resp[1024];
static size_t tmc_resp_len = 0;       // bytes in tmc_resp (text or block header)
static uint32_t tmc_wave_total = 0;   // waveform bytes after the header
static uint32_t tmc_wave_sent = 0;
static uint8_t tmc_wave_chan = 1;
static bool tmc_have_resp = false, tmc_in_requested = false, tmc_sending = false;
static uint8_t tmc_chunk[512];
static size_t tmc_text_sent = 0;
// Scope state.
static bool ds_disp[2] = { true, true };
static float ds_tim = 1e-3f, ds_scal[2] = { 1.0f, 2.0f }, ds_offs[2] = { 0.0f, -1.0f }, ds_lev = 0.5f, ds_tim_offs = 0.0f;
static int ds_prob[2] = { 10, 10 };
static const char *ds_coup[2] = { "DC", "AC" };
static char ds_sour[8] = "CH1", ds_slop[10] = "POSITIVE";
static bool ds_raw = false;

// "%.3e" by hand: the core's printf lacks the exponent forms.
static void fmt_sci(char *out, size_t n, float v) {
  int neg = v < 0;
  if (neg) v = -v;
  int e = 0;
  if (v > 0) {
    while (v >= 10.0f) { v /= 10.0f; e++; }
    while (v < 1.0f) { v *= 10.0f; e--; }
  }
  int m = (int)(v * 1000.0f + 0.5f);
  if (m >= 10000) { m /= 10; e++; }
  snprintf(out, n, "%s%d.%03de%c%02d\n", neg ? "-" : "", m / 1000, m % 1000, e < 0 ? '-' : '+', e < 0 ? -e : e);
}

static uint8_t ds_sample(uint8_t chan, uint32_t i) {
  if (chan == 1) {
    // One sine period per 600 points, 100 counts of amplitude.
    float v = sinf((float)(i % 600) * 6.2831853f / 600.0f);
    return (uint8_t)(128 - (int)(v * 100.0f));
  }
  return (uint8_t)(128 - (int)((i % 200) - 100));
}

static void tmc_text(const char *t) {
  tmc_resp_len = strlen(t);
  memcpy(tmc_resp, t, tmc_resp_len);
  tmc_wave_total = 0;
  tmc_have_resp = true;
}

static void tmc_handle(const char *cmd) {
  char out[64];
  int ch = 0;
  tmc_have_resp = false;
  if (!strcmp(cmd, "*IDN?")) { tmc_text("Rigol Technologies,DS1102E,DS1EB104702974,00.04.02\n"); return; }
  if (!strcmp(cmd, "*OPC?")) { tmc_text("1\n"); return; }
  if (!strcmp(cmd, ":TRIG:STAT?")) { tmc_text("T'D\n"); return; }
  if (!strcmp(cmd, ":TRIG:MODE?")) { tmc_text("EDGE\n"); return; }
  if (!strcmp(cmd, ":TIM:SCAL?")) { fmt_sci(out, sizeof(out), ds_tim); tmc_text(out); return; }
  if (!strcmp(cmd, ":TIM:OFFS?")) { fmt_sci(out, sizeof(out), ds_tim_offs); tmc_text(out); return; }
  if (!strcmp(cmd, ":TRIG:EDGE:SOUR?")) { snprintf(out, sizeof(out), "%s\n", ds_sour); tmc_text(out); return; }
  if (!strcmp(cmd, ":TRIG:EDGE:SLOP?")) { snprintf(out, sizeof(out), "%s\n", ds_slop); tmc_text(out); return; }
  if (!strcmp(cmd, ":TRIG:EDGE:LEV?")) { fmt_sci(out, sizeof(out), ds_lev); tmc_text(out); return; }
  if (sscanf(cmd, ":CHAN%d:", &ch) == 1 && ch >= 1 && ch <= 2) {
    const char *rest = strchr(cmd + 6, ':') + 1;
    float f; int n;
    if (!strcmp(rest, "DISP?")) { tmc_text(ds_disp[ch - 1] ? "ON\n" : "OFF\n"); return; }
    if (!strcmp(rest, "PROB?")) { snprintf(out, sizeof(out), "%d\n", ds_prob[ch - 1]); tmc_text(out); return; }
    if (!strcmp(rest, "SCAL?")) { fmt_sci(out, sizeof(out), ds_scal[ch - 1]); tmc_text(out); return; }
    if (!strcmp(rest, "OFFS?")) { fmt_sci(out, sizeof(out), ds_offs[ch - 1]); tmc_text(out); return; }
    if (!strcmp(rest, "COUP?")) { snprintf(out, sizeof(out), "%s\n", ds_coup[ch - 1]); tmc_text(out); return; }
    if (!strncmp(rest, "DISP ", 5)) { ds_disp[ch - 1] = !strcmp(rest + 5, "ON"); return; }
    if (sscanf(rest, "SCAL %f", &f) == 1) { ds_scal[ch - 1] = f; return; }
    if (sscanf(rest, "PROB %d", &n) == 1) { ds_prob[ch - 1] = n; return; }
    if (!strncmp(rest, "COUP ", 5)) { ds_coup[ch - 1] = !strcmp(rest + 5, "AC") ? "AC" : !strcmp(rest + 5, "GND") ? "GND" : "DC"; return; }
    return;
  }
  float f;
  if (sscanf(cmd, ":TIM:SCAL %f", &f) == 1) { ds_tim = f; return; }
  if (sscanf(cmd, ":TIM:OFFS %f", &f) == 1) { ds_tim_offs = f; return; }
  if (sscanf(cmd, ":TRIG:EDGE:LEV %f", &f) == 1) { ds_lev = f; return; }
  if (!strncmp(cmd, ":TRIG:EDGE:SOUR ", 16)) { snprintf(ds_sour, sizeof(ds_sour), "%s", !strcmp(cmd + 16, "CHAN1") ? "CH1" : !strcmp(cmd + 16, "CHAN2") ? "CH2" : cmd + 16); return; }
  if (!strncmp(cmd, ":TRIG:EDGE:SLOP ", 16)) { snprintf(ds_slop, sizeof(ds_slop), "%s", !strcmp(cmd + 16, "NEG") ? "NEGATIVE" : "POSITIVE"); return; }
  if (!strcmp(cmd, ":WAV:POIN:MODE RAW")) { ds_raw = true; return; }
  if (!strcmp(cmd, ":WAV:POIN:MODE NORMAL")) { ds_raw = false; return; }
  if (sscanf(cmd, ":WAV:DATA? CHAN%d", &ch) == 1 && ch >= 1 && ch <= 2) {
    tmc_wave_chan = ch;
    tmc_wave_total = ds_raw ? 1048576u : 600u;
    tmc_wave_sent = 0;
    tmc_resp_len = snprintf((char *)tmc_resp, sizeof(tmc_resp), "#9%09lu", (unsigned long)tmc_wave_total);
    tmc_have_resp = true;
    return;
  }
  // Everything else is a setting without an answer.
}

// Sends the next piece of the response; the header first, then the
// waveform bytes, then the line feed.
static void tmc_send_next() {
  size_t n = 0;
  bool last = false;
  if (tmc_text_sent < tmc_resp_len) {
    n = min(tmc_resp_len - tmc_text_sent, sizeof(tmc_chunk));
    memcpy(tmc_chunk, tmc_resp + tmc_text_sent, n);
    tmc_text_sent += n;
    last = (tmc_text_sent == tmc_resp_len) && tmc_wave_total == 0;
  } else {
    // Waveform bytes; the DS1000 ends the message with the data, no
    // line feed (libsigrok reads none for this series).
    n = min((size_t)(tmc_wave_total - tmc_wave_sent), sizeof(tmc_chunk));
    for (size_t i = 0; i < n; i++) tmc_chunk[i] = ds_sample(tmc_wave_chan, tmc_wave_sent + i);
    tmc_wave_sent += n;
    last = tmc_wave_sent == tmc_wave_total;
  }
  tmc_sending = !last;
  if (last) { tmc_have_resp = false; tmc_in_requested = false; tmc_text_sent = 0; }
  tud_usbtmc_transmit_dev_msg_data(tmc_chunk, n, last, false);
}

extern "C" {
static usbtmc_response_capabilities_488_t const tmc_caps = {
  .USBTMC_status = USBTMC_STATUS_SUCCESS,
  .bcdUSBTMC = USBTMC_VERSION,
  .bmIntfcCapabilities = { .listenOnly = 0, .talkOnly = 0, .supportsIndicatorPulse = 0 },
  .bmDevCapabilities = { .canEndBulkInOnTermChar = 0 },
  .bcdUSB488 = USBTMC_488_VERSION,
  .bmIntfcCapabilities488 = { .supportsTrigger = 0, .supportsREN_GTL_LLO = 0, .is488_2 = 0 },
  .bmDevCapabilities488 = { .DT1 = 0, .RL1 = 0, .SR1 = 0, .SCPI = 1 },
};
usbtmc_response_capabilities_488_t const *tud_usbtmc_get_capabilities_cb(void) { return &tmc_caps; }
void tud_usbtmc_open_cb(uint8_t) { tud_usbtmc_start_bus_read(); }
bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const *) { tmc_cmd_len = 0; return true; }
bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete) {
  size_t room = sizeof(tmc_cmd) - 1 - tmc_cmd_len;
  size_t n = min(len, room);
  memcpy(tmc_cmd + tmc_cmd_len, data, n);
  tmc_cmd_len += n;
  if (transfer_complete) {
    while (tmc_cmd_len && (tmc_cmd[tmc_cmd_len - 1] == '\n' || tmc_cmd[tmc_cmd_len - 1] == '\r')) tmc_cmd_len--;
    tmc_cmd[tmc_cmd_len] = 0;
    tmc_handle(tmc_cmd);
    tmc_cmd_len = 0;
    tmc_text_sent = 0;
  }
  tud_usbtmc_start_bus_read();
  return true;
}
// Every piece of a response answers one host request: the host asks
// again while a message ends without the end flag.
bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const *) {
  tmc_in_requested = true;
  if (tmc_have_resp) tmc_send_next();
  return true;
}
bool tud_usbtmc_msgBulkIn_complete_cb(void) {
  tud_usbtmc_start_bus_read();
  return true;
}
void tud_usbtmc_bulkOut_clearFeature_cb(void) { tud_usbtmc_start_bus_read(); }
void tud_usbtmc_bulkIn_clearFeature_cb(void) {}
bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *r) { tmc_sending = false; tmc_have_resp = false; *r = USBTMC_STATUS_SUCCESS; return true; }
bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *r) { *r = USBTMC_STATUS_SUCCESS; return true; }
bool tud_usbtmc_initiate_clear_cb(uint8_t *r) { tmc_sending = false; tmc_have_resp = false; tmc_in_requested = false; *r = USBTMC_STATUS_SUCCESS; return true; }
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *) { tud_usbtmc_start_bus_read(); return true; }
bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *) { tud_usbtmc_start_bus_read(); return true; }
bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp) {
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0;
  return true;
}
uint8_t tud_usbtmc_get_stb_cb(uint8_t *r) { *r = USBTMC_STATUS_SUCCESS; return 0; }
}

static void pk_fill() {
  for (unsigned i = 0; i < 512; i++) {
    unsigned j = 511 - i;
    pk_raw[i] = pk_nibble(2 * j) | (pk_nibble(2 * j + 1) << 4);
  }
}

static void load() {
  EEPROM.begin(256);
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC || cfg.packet_len > sizeof(cfg.packet)) {
    cfg.magic = MAGIC;
    cfg.mode = BU86X;
    cfg.period_ms = 0;
    cfg.packet_len = sizeof(default_bm86x);
    memcpy(cfg.packet, default_bm86x, sizeof(default_bm86x));
  }
}

static void save() {
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

static const char *mode_name(uint8_t m) {
  switch (m) {
    case CH9325: return "ch9325";
    case VICTOR: return "victor";
    case USBRELAY: return "usbrelay";
    case PICKIT2: return "pickit2";
    case USBTMC: return "usbtmc";
    default: return "bu86x";
  }
}

static uint8_t bit_reverse(uint8_t b) {
  b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
  b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
  b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
  return b;
}

// Waits for the IN endpoint, up to 50 ms.
static bool wait_ready() {
  for (int i = 0; i < 50 && !usb_hid.ready(); i++) delay(1);
  return usb_hid.ready();
}

// Sends the packet the way the selected cable would.
static void emit() {
  uint8_t rep[64];
  switch (cfg.mode) {
    case BU86X:
      for (size_t pos = 0; pos < cfg.packet_len; pos += 8) {
        size_t n = min((size_t)8, cfg.packet_len - pos);
        memset(rep, 0, 8);
        memcpy(rep, cfg.packet + pos, n);
        if (!wait_ready()) return;
        usb_hid.sendReport(0, rep, 8);
      }
      break;
    case CH9325:
      for (size_t pos = 0; pos < cfg.packet_len; pos += 7) {
        size_t n = min((size_t)7, cfg.packet_len - pos);
        memset(rep, 0, 8);
        rep[0] = 0xf0 | n;
        memcpy(rep + 1, cfg.packet + pos, n);
        if (!wait_ready()) return;
        usb_hid.sendReport(0, rep, 8);
      }
      break;
    case VICTOR: {
      // The cable's scrambling, see libsigrok serial_hid_victor.c:
      // byte order shuffled, a fixed offset added, bits reversed.
      static const uint8_t obfuscation[14] = {
        'j', 'o', 'd', 'e', 'n', 'x', 'u', 'n', 'i', 'c', 'k', 'x', 'i', 'a',
      };
      static const uint8_t shuffle[14] = { 6, 13, 5, 11, 2, 7, 9, 8, 3, 10, 12, 0, 4, 1 };
      uint8_t plain[14];
      memset(plain, 0, sizeof(plain));
      memcpy(plain, cfg.packet, min((size_t)14, cfg.packet_len));
      for (int i = 0; i < 14; i++)
        rep[i] = bit_reverse(plain[13 - shuffle[i]]) + obfuscation[i];
      if (!wait_ready()) return;
      usb_hid.sendReport(0, rep, 14);
      break;
    }
  }
}

static uint16_t get_report_cb(uint8_t, hid_report_type_t type, uint8_t *buf, uint16_t len) {
  if (type != HID_REPORT_TYPE_FEATURE) return 0;
  if (cfg.mode == USBRELAY) {
    uint8_t rep[8] = { 'A', 'B', 'C', 'D', 'E', 0, 0, relay_state };
    size_t n = min((size_t)len, sizeof(rep));
    memcpy(buf, rep, n);
    return n;
  }
  size_t n = min((size_t)len, last_feature_len);
  memcpy(buf, last_feature, n);
  return n;
}

static void set_report_cb(uint8_t, hid_report_type_t type, uint8_t const *buf, uint16_t len) {
  if (type == HID_REPORT_TYPE_FEATURE) {
    last_feature_len = min((size_t)len, sizeof(last_feature));
    memcpy(last_feature, buf, last_feature_len);
    if (cfg.mode == USBRELAY && len >= 2) {
      switch (buf[0]) {
        case 0xfe: relay_state = 0xff; break;
        case 0xfc: relay_state = 0x00; break;
        case 0xff: if (buf[1] >= 1 && buf[1] <= 8) relay_state |= 1 << (buf[1] - 1); break;
        case 0xfd: if (buf[1] >= 1 && buf[1] <= 8) relay_state &= ~(1 << (buf[1] - 1)); break;
      }
    }
    Serial.print("FEATURE");
    for (size_t i = 0; i < last_feature_len; i++) Serial.printf(" %02x", buf[i]);
    Serial.println();
    return;
  }
  Serial.print("OUT");
  for (size_t i = 0; i < len; i++) Serial.printf(" %02x", buf[i]);
  Serial.println();
  // The Brymen adapter answers a request; the others ignore writes.
  if (cfg.mode == BU86X && len >= 3 && buf[2] == 0x66) emit_requested = true;
  if (cfg.mode == PICKIT2 && len >= 1) {
    switch (buf[0]) {
      case 0xb8: pk_status_due = millis() + 300; pk_status_pending = true; break;
      case 0xb9: pk_offset = buf[1]; pk_bank = buf[2] - 6; pk_reads = 0; break;
      case 0xac: pk_read_pending = true; break;
    }
  }
}

static int unhex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void status() {
  Serial.printf("MODE %s PERIOD %u PACKET ", mode_name(cfg.mode), cfg.period_ms);
  for (int i = 0; i < cfg.packet_len; i++) Serial.printf("%02x", cfg.packet[i]);
  Serial.println();
}

static void command(String line) {
  line.trim();
  if (line.startsWith("MODE ")) {
    String m = line.substring(5);
    m.trim();
    uint8_t mode = cfg.mode;
    if (m == "bu86x") mode = BU86X;
    else if (m == "ch9325") mode = CH9325;
    else if (m == "victor") mode = VICTOR;
    else if (m == "usbrelay") mode = USBRELAY;
    else if (m == "pickit2") mode = PICKIT2;
    else if (m == "usbtmc") mode = USBTMC;
    else { Serial.println("ERR unknown mode"); return; }
    cfg.mode = mode;
    save();
    Serial.println("OK rebooting");
    delay(100);
    rp2040.reboot();
  } else if (line.startsWith("PACKET ")) {
    String h = line.substring(7);
    h.trim();
    if (h.length() % 2 || h.length() / 2 > sizeof(cfg.packet)) { Serial.println("ERR packet"); return; }
    for (unsigned i = 0; i < h.length(); i += 2) {
      int a = unhex(h[i]), b = unhex(h[i + 1]);
      if (a < 0 || b < 0) { Serial.println("ERR hex"); return; }
      cfg.packet[i / 2] = a << 4 | b;
    }
    cfg.packet_len = h.length() / 2;
    Serial.println("OK");
  } else if (line.startsWith("PERIOD ")) {
    cfg.period_ms = line.substring(7).toInt();
    Serial.println("OK");
  } else if (line == "STATUS") {
    status();
  } else if (line == "SAVE") {
    save();
    Serial.println("OK");
  } else if (line == "REBOOT") {
    rp2040.reboot();
  } else if (line == "BOOTSEL") {
    rp2040.rebootToBootloader();
  } else if (line == "EMIT") {
    emit();
    Serial.println("OK");
  } else if (line.length()) {
    Serial.println("ERR commands: MODE bu86x|ch9325|victor|usbrelay|pickit2|usbtmc, PACKET hex, PERIOD ms, EMIT, STATUS, SAVE, REBOOT, BOOTSEL");
  }
}

void setup() {
  load();
  switch (cfg.mode) {
    case CH9325:
      TinyUSBDevice.setID(0x1a86, 0xe008);
      TinyUSBDevice.setManufacturerDescriptor("WCH");
      TinyUSBDevice.setProductDescriptor("CH9325 cable emulator");
      usb_hid.setReportDescriptor(desc_8, sizeof(desc_8));
      break;
    case VICTOR:
      TinyUSBDevice.setID(0x1244, 0xd237);
      TinyUSBDevice.setManufacturerDescriptor("Victor");
      TinyUSBDevice.setProductDescriptor("Victor cable emulator");
      usb_hid.setReportDescriptor(desc_14, sizeof(desc_14));
      break;
    case USBRELAY:
      TinyUSBDevice.setID(0x16c0, 0x05df);
      TinyUSBDevice.setManufacturerDescriptor("www.dcttech.com");
      TinyUSBDevice.setProductDescriptor("USBRelay2");
      usb_hid.setReportDescriptor(desc_8, sizeof(desc_8));
      break;
    case USBTMC:
      TinyUSBDevice.setID(0x1ab1, 0x0588);
      TinyUSBDevice.setManufacturerDescriptor("Rigol Technologies");
      TinyUSBDevice.setProductDescriptor("DS1000 SERIES");
      break;
    case PICKIT2:
      TinyUSBDevice.setID(0x04d8, 0x0033);
      TinyUSBDevice.setManufacturerDescriptor("Microchip Technology Inc.");
      TinyUSBDevice.setProductDescriptor("PICkit 2 Microcontroller Programmer");
      usb_hid.setReportDescriptor(desc_64, sizeof(desc_64));
      pk_fill();
      break;
    default:
      TinyUSBDevice.setID(0x0820, 0x0001);
      TinyUSBDevice.setManufacturerDescriptor("Brymen");
      TinyUSBDevice.setProductDescriptor("BU-86X emulator");
      usb_hid.setReportDescriptor(desc_8, sizeof(desc_8));
      break;
  }
  if (cfg.mode == USBTMC) {
    TinyUSBDevice.setSerialDescriptor("DS1EB104702974");
    TinyUSBDevice.addInterface(usbtmc_itf);
  } else {
    TinyUSBDevice.setSerialDescriptor("HIDCABLE1");
    usb_hid.setPollInterval(2);
    usb_hid.setReportCallback(get_report_cb, set_report_cb);
    usb_hid.begin();
  }
  Serial.begin(115200);
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      command(line);
      line = "";
    } else if (line.length() < 200) {
      line += c;
    }
  }
  if (emit_requested) {
    emit_requested = false;
    emit();
  }
  if (pk_status_pending && (long)(millis() - pk_status_due) >= 0) {
    pk_status_pending = false;
    uint8_t rep[64];
    memset(rep, 0, sizeof(rep));
    if (wait_ready()) usb_hid.sendReport(0, rep, 64);
  }
  if (pk_read_pending) {
    pk_read_pending = false;
    unsigned at = (pk_bank & 1) * 256 + pk_offset + pk_reads * 64;
    pk_reads++;
    uint8_t rep[64];
    for (unsigned i = 0; i < 64; i++) rep[i] = pk_raw[(at + i) % 512];
    if (wait_ready()) usb_hid.sendReport(0, rep, 64);
  }
  if (cfg.period_ms && millis() - last_emit >= cfg.period_ms) {
    last_emit = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    emit();
  }
}
