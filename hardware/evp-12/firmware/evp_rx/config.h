// EVP-12 Intersection Controller Unit (CU) — per-intersection settings
#pragma once
#include "src/evp_logic.h"

#define INTERSECTION_NAME  "MHD-DEMO-01"
#define SYSTEM_CODE        0x5A
#define FLEET_KEY { \
  0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10, \
  0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20 }

// Fleet vehicle ids whose last counter is persisted for replay protection across reboots
static const uint16_t FLEET_VIDS[] = {0x0A34, 0x0A35, 0x0A36};

// ---- approaches -------------------------------------------------------------------
// One entry per approach. stop_lat/lon = middle of that approach's stop line (measure on site
// or from a map). inbound_bearing = direction vehicles DRIVE on this approach toward the
// junction (0 = northbound, 90 = eastbound, 180 = southbound, 270 = westbound).
// ir_channel = which detector head input (0..3) looks down this approach.
// EXAMPLE values only — replace with the real intersection.
static const evp::Approach APPROACHES[] = {
  // name  stop_lat    stop_lon    bearing  half_w  max_d  ir_ch
  {"N",   36.297120,  59.606200,  180.f,   25.f,   600.f,  0},   // comes from the north, drives south
  {"E",   36.297000,  59.606350,  270.f,   25.f,   600.f,  1},
  {"S",   36.296880,  59.606200,    0.f,   25.f,   600.f,  2},
  {"W",   36.297000,  59.606050,   90.f,   25.f,   600.f,  3},
};
static const int N_APPROACHES = sizeof(APPROACHES) / sizeof(APPROACHES[0]);

static evp::Config make_config() {
  evp::Config c;
  c.system_code      = SYSTEM_CODE;
  c.ir_only_allowed  = true;
  c.gps_only_allowed = true;   // early trigger from signed GPS (set false to require IR)
  c.gps_eta_s        = 25.f;
  c.pre_eta_s        = 45.f;
  c.max_request_ms   = 120000;
  return c;
}

// ---- pins (ESP32-WROOM-32E DevKitC) ------------------------------------------------
// detector inputs: opto collector, LOW = IR burst; external 4.7k pull-ups to 3V3 (input-only pins)
static const int PIN_IR[4]     = {34, 35, 36, 39};
static const int PIN_RELAY[4]  = {25, 26, 27, 13};   // per approach (index = APPROACHES order)
#define PIN_RELAY_HEALTHY 32                         // energised while the unit is healthy
#define RELAY_ON_LEVEL    HIGH                       // ULN2003 driver. Many relay boards: LOW
#define PIN_LORA_NSS   5
#define PIN_LORA_RST   22
#define PIN_LORA_DIO0  21
#define LORA_FREQ_HZ   433E6
#define PIN_RS485_TX   17
#define PIN_RS485_RX   16
#define PIN_RS485_DE   4                             // DE and /RE tied together
#define RS485_BAUD     115200
#define PIN_LED        2
