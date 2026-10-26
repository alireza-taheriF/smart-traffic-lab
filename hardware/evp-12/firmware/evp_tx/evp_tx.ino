// EVP-12 Vehicle Transmitter Unit (VTU)
// Board: ESP32 Dev Module (Arduino-ESP32 core 3.x)
// Libraries: "LoRa" by Sandeep Mistry, "TinyGPSPlus" by Mikal Hart
//
// Every 83.333 ms (12 Hz) sends one 32-bit IR frame on a 38 kHz carrier through the RMT
// peripheral, cycling the emitter power HIGH/MID/HIGH/LOW/HIGH/MID. Every second sends a
// signed LoRa packet with GPS position, heading, speed and a monotonic counter.
// Only transmits while the lights/siren input is active.

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include "config.h"
#include "src/evp_proto.h"

using namespace evp;

static const uint8_t KEY[32] = FLEET_KEY;
static TinyGPSPlus gps;
static Preferences prefs;
static volatile uint32_t g_counter = 0;      // signed counter (LoRa); IR copies its low 12 bits
static uint32_t g_saved_until = 0;
static volatile bool g_active = false;
static bool g_lora_ok = false;
static const uint8_t PATTERN[6] = {LVL_HIGH, LVL_MID, LVL_HIGH, LVL_LOW, LVL_HIGH, LVL_MID};

// ---------------------------------------------------------------- counter (never goes back)
static void counter_load() {
  prefs.begin("evp", false);
  g_counter = prefs.getULong("ctr", 1);
  g_saved_until = g_counter;
}
static uint32_t counter_next() {
  uint32_t c = ++g_counter;
  if (c >= g_saved_until) {                  // reserve the next 64 values in flash
    g_saved_until = c + 64;
    prefs.putULong("ctr", g_saved_until);
  }
  return c;
}

// ---------------------------------------------------------------- IR emitter
static void set_level(uint8_t lvl) {
  digitalWrite(PIN_EN_A, lvl == LVL_HIGH);
  digitalWrite(PIN_EN_B, lvl == LVL_HIGH || lvl == LVL_MID);
  digitalWrite(PIN_EN_C, lvl == LVL_LOW);
}

static void ir_task(void*) {
  rmt_data_t sym[40];
  uint16_t dur[80];
  uint32_t n = 0;
  int64_t next = esp_timer_get_time();
  for (;;) {
    next += IR_PERIOD_US;
    int64_t wait = next - esp_timer_get_time();
    if (wait > 2500) vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
    while (esp_timer_get_time() < next) { /* fine wait (< 2 ms) */ }
    if (!g_active) { set_level(0xFF); n = 0; continue; }

    IrFrame f{PATTERN[n % 6], (uint8_t)VEHICLE_PRIO, (uint8_t)(VEHICLE_ID & 0xFF), (uint16_t)(g_counter & 0x0FFF)};
    size_t k = ir_build_symbols(ir_pack(f, SYSTEM_CODE), dur, 80);   // mark, space, ..., mark
    size_t m = 0;
    for (size_t i = 0; i < k; i += 2) {
      sym[m].level0 = 1; sym[m].duration0 = dur[i];                  // mark: carrier ON
      sym[m].level1 = 0; sym[m].duration1 = (i + 1 < k) ? dur[i + 1] : 10;
      ++m;
    }
    set_level(f.level);
    rmtWrite(PIN_IR_DATA, sym, m, RMT_WAIT_FOR_EVER);                 // ~25-42 ms
    set_level(0xFF);                                                  // all gates closed
    ++n;
  }
}

// ---------------------------------------------------------------- LoRa beacon
static void send_beacon() {
  LoraPkt p;
  bool fix = gps.location.isValid() && gps.location.age() < 2000;
  p.flags = F_ACTIVE | F_IR_ON | (fix ? F_GPS_FIX : 0);
  p.vid = VEHICLE_ID; p.prio = VEHICLE_PRIO;
  p.sats = gps.satellites.isValid() ? (uint8_t)min<uint32_t>(gps.satellites.value(), 255) : 0;
  p.counter = counter_next();
  if (fix) {
    p.lat_e7 = (int32_t)llround(gps.location.lat() * 1e7);
    p.lon_e7 = (int32_t)llround(gps.location.lng() * 1e7);
  }
  p.heading_cdeg = gps.course.isValid() ? (uint16_t)(fmod(gps.course.deg(), 360.0) * 100) : 0;
  p.speed_dms = gps.speed.isValid() ? (uint16_t)min(65535.0, gps.speed.mps() * 10) : 0;
  p.hdop_d = gps.hdop.isValid() ? (uint8_t)min(255.0, gps.hdop.hdop() * 10) : 255;
  uint8_t buf[LORA_PKT_LEN];
  lora_encode(p, KEY, sizeof KEY, buf);
  LoRa.beginPacket();
  LoRa.write(buf, sizeof buf);
  LoRa.endPacket();                         // blocking, ~70 ms at SF7/125k
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN_A, OUTPUT); pinMode(PIN_EN_B, OUTPUT); pinMode(PIN_EN_C, OUTPUT);
  set_level(0xFF);
  pinMode(PIN_ENABLE_IN, INPUT);            // GPIO34 has no internal pull-up: fit 10k to 3V3
  pinMode(PIN_STATUS_LED, OUTPUT);

  counter_load();
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // RMT: 1 us ticks, 38 kHz carrier, 25 % duty, carrier applied to the HIGH (mark) level.
  // Bench check with a scope or a phone camera: carrier_level=true modulates the HIGH level.
  if (!rmtInit(PIN_IR_DATA, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000) ||
      !rmtSetCarrier(PIN_IR_DATA, true, true, IR_CARRIER_HZ, 0.25f)) {
    Serial.println("RMT init failed");
  }
  rmtSetEOT(PIN_IR_DATA, LOW);

  LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
  g_lora_ok = LoRa.begin(LORA_FREQ_HZ);
  if (g_lora_ok) {
    LoRa.setSpreadingFactor(7); LoRa.setSignalBandwidth(125E3); LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12); LoRa.enableCrc(); LoRa.setTxPower(LORA_TX_DBM);
  } else Serial.println("LoRa init failed");

  xTaskCreatePinnedToCore(ir_task, "ir", 4096, nullptr, 5, nullptr, 1);
  Serial.printf("EVP-12 VTU vid=0x%04X prio=%d counter=%lu\n", VEHICLE_ID, VEHICLE_PRIO, (unsigned long)g_counter);
}

void loop() {
  static uint32_t t_beacon = 0, t_print = 0;
  while (Serial2.available()) gps.encode(Serial2.read());

  // lights/siren input, debounced 200 ms
  static uint32_t t_change = 0; static bool raw_prev = false;
  bool raw = ENABLE_ALWAYS || digitalRead(PIN_ENABLE_IN) == LOW;
  if (raw != raw_prev) { raw_prev = raw; t_change = millis(); }
  if (millis() - t_change > 200) g_active = raw;

  uint32_t now = millis();
  if (g_active && g_lora_ok && now - t_beacon >= LORA_PERIOD_MS) { t_beacon = now; send_beacon(); }

  // cab LED: off = idle, slow blink = active without GPS, solid = active with GPS, fast = fault
  bool fix = gps.location.isValid() && gps.location.age() < 2000;
  bool led;
  if (!g_lora_ok)       led = (now / 100) % 2;
  else if (!g_active)   led = false;
  else if (!fix)        led = (now / 500) % 2;
  else                  led = true;
  digitalWrite(PIN_STATUS_LED, led);

  if (now - t_print > 2000) {
    t_print = now;
    Serial.printf("active=%d fix=%d sats=%lu hdop=%.1f ctr=%lu\n", g_active, fix,
                  (unsigned long)gps.satellites.value(), gps.hdop.hdop(), (unsigned long)g_counter);
  }
  delay(2);
}
