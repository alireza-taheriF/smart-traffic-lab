// EVP-12 Intersection Controller Unit (CU)
// Board: ESP32 Dev Module (Arduino-ESP32 core 3.x)
// Libraries: "LoRa" by Sandeep Mistry
//
// Decodes 12 Hz IR frames from up to 4 directional detector heads, verifies signed LoRa
// beacons, decides which ONE approach (if any) gets a preemption request, and drives:
//   - one dry-contact relay per approach -> preemption inputs of the existing controller
//   - a HEALTHY relay (drops on fault / watchdog reset)
//   - JSON lines on RS-485 for the smart-traffic-lab controller
// The unit never switches signal lamps itself.

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "src/evp_proto.h"
#include "src/evp_logic.h"

using namespace evp;

static const uint8_t KEY[32] = FLEET_KEY;
static Engine engine;
static Preferences prefs;
static IrDecoder dec[4];
static uint32_t seq = 0;
static int16_t last_rssi = 0;
static bool lora_ok = false, inhibit = false;
static bool head_fault[4] = {false, false, false, false};
static int test_app = -1; static uint32_t test_until = 0;
void out_line(const char* s);

// ---------------------------------------------------------------- edge capture (ISR)
struct Edge { uint32_t t_us; uint8_t level; };
static const int EQ = 256;
static volatile Edge edges[4][EQ];
static volatile uint16_t ehead[4] = {0}, etail[4] = {0};

static void IRAM_ATTR on_edge(void* arg) {
  int ch = (int)(intptr_t)arg;
  uint16_t h = ehead[ch], nh = (h + 1) % EQ;
  if (nh == etail[ch]) return;                       // overflow: drop
  edges[ch][h].t_us = (uint32_t)esp_timer_get_time();
  edges[ch][h].level = (uint8_t)gpio_get_level((gpio_num_t)PIN_IR[ch]);
  ehead[ch] = nh;
}

static uint32_t last_edge_us[4] = {0}; static uint8_t last_level[4] = {1, 1, 1, 1};

static void drain_ir() {
  for (int ch = 0; ch < 4; ++ch) {
    while (etail[ch] != ehead[ch]) {
      uint16_t t = etail[ch];
      uint32_t ts = edges[ch][t].t_us; uint8_t lv = edges[ch][t].level;
      etail[ch] = (t + 1) % EQ;
      uint32_t dur = ts - last_edge_us[ch];
      bool was_mark = last_level[ch] == 0;               // LOW = IR burst present
      if (dec[ch].feed(was_mark, dur, ts)) {
        IrFrame f;
        if (ir_unpack(dec[ch].word(), SYSTEM_CODE, f))
        {
          uint32_t age_us = (uint32_t)esp_timer_get_time() - dec[ch].frame_start_us();
          engine.on_ir_frame(ch, f, millis() - age_us / 1000);
        }
      }
      last_edge_us[ch] = ts; last_level[ch] = lv;
    }
    // stuck-LOW line (shorted cable / failed head) -> fault
    bool low_now = gpio_get_level((gpio_num_t)PIN_IR[ch]) == 0;
    uint32_t held = (uint32_t)esp_timer_get_time() - last_edge_us[ch];
    bool fault = low_now && held > 500000;
    if (fault != head_fault[ch]) {
      head_fault[ch] = fault;
      char b[96]; snprintf(b, sizeof b, "{\"src\":\"EVP12\",\"seq\":%lu,\"type\":\"HEAD_FAULT\",\"ch\":%d,\"on\":%s}",
                           (unsigned long)++seq, ch, fault ? "true" : "false");
      out_line(b);
    }
  }
}

// ---------------------------------------------------------------- outputs
void out_line(const char* s) {
  Serial.println(s);
  digitalWrite(PIN_RS485_DE, HIGH);
  Serial2.println(s);
  Serial2.flush();
  digitalWrite(PIN_RS485_DE, LOW);
}

static const char* app_name(int a) { return (a >= 0 && a < N_APPROACHES) ? APPROACHES[a].name : "-"; }
static const char* lvl_name(uint8_t l) { return l == LVL_LOW ? "LOW" : l == LVL_MID ? "MID" : l == LVL_HIGH ? "HIGH" : "-"; }

static void on_event(Event e, const Request& r, const char* why) {
  static const char* names[] = {"REQ_ON", "REQ_OFF", "PRE", "DIR_CONFLICT", "REPLAY", "BAD_MAC"};
  char b[256];
  snprintf(b, sizeof b,
           "{\"src\":\"EVP12\",\"seq\":%lu,\"type\":\"%s\",\"app\":\"%s\",\"vid\":%u,\"prio\":%u,\"via\":\"%s\","
           "\"eta_s\":%.1f,\"dist_m\":%.0f,\"lvl\":\"%s\",\"why\":\"%s\"}",
           (unsigned long)++seq, names[(int)e], app_name(r.approach), r.vid, r.prio, source_name(r.src),
           r.eta_s, r.dist_m, lvl_name(r.ir_level), why);
  out_line(b);
  if (e == Event::REQ_ON) {                          // persist this vehicle's counter (rare write)
    const Engine::Track* t = engine.track(r.vid);
    if (t) { char k[12]; snprintf(k, sizeof k, "c%04x", r.vid); prefs.putULong(k, t->counter); }
  }
}

static void apply_relays(uint32_t now) {
  const Request& r = engine.request();
  for (int i = 0; i < N_APPROACHES && i < 4; ++i) {
    bool on = (r.active && r.approach == i) || (!r.active && test_app == i && now < test_until);
    digitalWrite(PIN_RELAY[i], on ? RELAY_ON_LEVEL : !RELAY_ON_LEVEL);
  }
  bool any_fault = !lora_ok;
  for (int i = 0; i < 4; ++i) any_fault |= head_fault[i];
  digitalWrite(PIN_RELAY_HEALTHY, any_fault ? !RELAY_ON_LEVEL : RELAY_ON_LEVEL);
}

static void handle_cmd(const char* s) {
  if (strstr(s, "\"inhibit\"")) { inhibit = strstr(s, "true") != nullptr; engine.inhibit(inhibit); }
  else if (strstr(s, "\"test\"")) {
    for (int i = 0; i < N_APPROACHES; ++i) {
      char k[16]; snprintf(k, sizeof k, "\"%s\"", APPROACHES[i].name);
      const char* a = strstr(s, "\"app\"");
      if (a && strstr(a, k)) { test_app = i; test_until = millis() + 2000; }
    }
  }
}

static void read_cmds(Stream& in, char* buf, size_t& n) {
  while (in.available()) {
    char c = in.read();
    if (c == '\n' || n >= 159) { buf[n] = 0; if (n) handle_cmd(buf); n = 0; }
    else if (c != '\r') buf[n++] = c;
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  for (int i = 0; i < 4; ++i) { digitalWrite(PIN_RELAY[i], !RELAY_ON_LEVEL); pinMode(PIN_RELAY[i], OUTPUT); }
  digitalWrite(PIN_RELAY_HEALTHY, !RELAY_ON_LEVEL); pinMode(PIN_RELAY_HEALTHY, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RS485_DE, OUTPUT); digitalWrite(PIN_RS485_DE, LOW);
  Serial.begin(115200);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  static Approach apps[4];
  for (int i = 0; i < N_APPROACHES && i < 4; ++i) apps[i] = APPROACHES[i];
  engine.begin(make_config(), apps, N_APPROACHES, KEY, sizeof KEY, on_event);

  prefs.begin("evp", false);
  for (uint16_t v : FLEET_VIDS) {
    char k[12]; snprintf(k, sizeof k, "c%04x", v);
    uint32_t c = prefs.getULong(k, 0);
    if (c) engine.seed_counter(v, c);
  }

  for (int ch = 0; ch < 4; ++ch) {
    pinMode(PIN_IR[ch], INPUT);                      // external 4.7k pull-up
    dec[ch].reset();
    last_edge_us[ch] = (uint32_t)esp_timer_get_time();
    attachInterruptArg(PIN_IR[ch], on_edge, (void*)(intptr_t)ch, CHANGE);
  }

  LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
  lora_ok = LoRa.begin(LORA_FREQ_HZ);
  if (lora_ok) {
    LoRa.setSpreadingFactor(7); LoRa.setSignalBandwidth(125E3); LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12); LoRa.enableCrc();
  }
  esp_task_wdt_add(NULL);                            // core's task watchdog resets a hung loop
  char b[128]; snprintf(b, sizeof b, "{\"src\":\"EVP12\",\"type\":\"BOOT\",\"x\":\"%s\",\"lora\":%s,\"approaches\":%d}",
                        INTERSECTION_NAME, lora_ok ? "true" : "false", N_APPROACHES);
  out_line(b);
}

void loop() {
  static uint32_t t_tick = 0, t_hb = 0;
  static char cb1[160], cb2[160]; static size_t n1 = 0, n2 = 0;
  esp_task_wdt_reset();
  drain_ir();

  if (lora_ok) {
    int sz = LoRa.parsePacket();
    if (sz > 0) {
      uint8_t buf[64]; int n = 0;
      while (LoRa.available() && n < (int)sizeof buf) buf[n++] = LoRa.read();
      last_rssi = LoRa.packetRssi();
      engine.on_lora(buf, n, millis());
    }
  }

  uint32_t now = millis();
  if (now - t_tick >= 20) { t_tick = now; engine.tick(now); apply_relays(now); }

  read_cmds(Serial2, cb1, n1);
  read_cmds(Serial, cb2, n2);

  if (now - t_hb >= 1000) {
    t_hb = now;
    const Request& r = engine.request();
    char b[200];
    snprintf(b, sizeof b, "{\"src\":\"EVP12\",\"seq\":%lu,\"type\":\"HB\",\"up_s\":%lu,\"req\":\"%s\",\"heads\":[%d,%d,%d,%d],\"rssi\":%d,\"inhibit\":%s}",
             (unsigned long)++seq, (unsigned long)(now / 1000), r.active ? app_name(r.approach) : "-",
             !head_fault[0], !head_fault[1], !head_fault[2], !head_fault[3], last_rssi, inhibit ? "true" : "false");
    out_line(b);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
}
