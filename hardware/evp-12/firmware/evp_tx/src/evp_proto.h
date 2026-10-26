// EVP-12 — Emergency Vehicle Preemption, 12 Hz directional IR + signed LoRa/GPS
// Shared protocol code (portable C++11: builds for ESP32/Arduino and on a PC for tests).
// License: PolyForm Noncommercial 1.0.0 (same as smart-traffic-lab)
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace evp {

// ---------------------------------------------------------------------------
// IR frame (optical channel, 38 kHz carrier, 12 frames per second)
// ---------------------------------------------------------------------------
// 32-bit word, MSB first:
//   [31:30] level  0=LOW 1=MID 2=HIGH (emitter power used for this frame)
//   [29:28] prio   0..3 (3 = highest)
//   [27:20] vid8   low 8 bits of vehicle id
//   [19:8]  rc12   low 12 bits of the vehicle's signed LoRa counter
//   [7:0]   crc8   CRC-8/0x07 over the upper 24 bits, XOR system code
// Timing (T = 300 us, ~11 carrier cycles at 38 kHz):
//   header: mark 6T, space 6T
//   bit:    mark 1T, space 1T (0) or 3T (1)
//   stop:   mark 1T
// Longest frame = 12T + 32*4T + 1T = 141T = 42.3 ms, inside the 83.3 ms period.
// Satisfies TSOP382xx (AGC2) rules: bursts >= 10 cycles, bursts <= 70 cycles,
// gaps >= 10 cycles, ~410 bursts/s (< 1800/s).

constexpr uint32_t IR_T_US          = 300;
constexpr uint32_t IR_HDR_MARK_US   = 6 * IR_T_US;
constexpr uint32_t IR_HDR_SPACE_US  = 6 * IR_T_US;
constexpr uint32_t IR_FRAME_RATE_HZ = 12;
constexpr uint32_t IR_PERIOD_US     = 1000000UL / IR_FRAME_RATE_HZ;   // 83333
constexpr uint32_t IR_CARRIER_HZ    = 38000;

enum IrLevel : uint8_t { LVL_LOW = 0, LVL_MID = 1, LVL_HIGH = 2 };

struct IrFrame {
  uint8_t  level;
  uint8_t  prio;
  uint8_t  vid8;
  uint16_t rc12;
};

uint8_t  crc8_07(const uint8_t* d, size_t n);
uint32_t ir_pack(const IrFrame& f, uint8_t system_code);
bool     ir_unpack(uint32_t word, uint8_t system_code, IrFrame& out);

// Builds the mark/space duration list for one frame. out[] alternates
// mark, space, mark, space ... starting with a mark. Returns count (67).
size_t ir_build_symbols(uint32_t word, uint16_t* out, size_t max);

// Edge-driven decoder for one receiver channel. Feed every finished segment
// (mark = receiver output active). Returns true when a full frame finished.
class IrDecoder {
 public:
  void reset();
  // dur_us: segment length; end_us: timestamp at the end of the segment
  bool feed(bool is_mark, uint32_t dur_us, uint32_t end_us);
  uint32_t word() const { return word_; }
  uint32_t frame_start_us() const { return start_us_; }
 private:
  enum St : uint8_t { IDLE, HDR_SPACE, BIT_MARK, BIT_SPACE, STOP_MARK };
  St st_ = IDLE;
  uint32_t word_ = 0, acc_ = 0, start_us_ = 0;
  uint8_t nbits_ = 0;
};

// ---------------------------------------------------------------------------
// SHA-256 / HMAC-SHA256 (small, dependency-free)
// ---------------------------------------------------------------------------
struct Sha256 {
  uint32_t h[8];
  uint8_t  buf[64];
  uint64_t len;
  size_t   blen;
  void init();
  void update(const uint8_t* d, size_t n);
  void final(uint8_t out[32]);
};
void hmac_sha256(const uint8_t* key, size_t klen, const uint8_t* msg, size_t mlen, uint8_t out[32]);

// ---------------------------------------------------------------------------
// LoRa packet (radio channel, ~1 Hz)  — 34 bytes, little-endian
// ---------------------------------------------------------------------------
constexpr size_t LORA_BODY_LEN = 26;
constexpr size_t LORA_MAC_LEN  = 8;
constexpr size_t LORA_PKT_LEN  = LORA_BODY_LEN + LORA_MAC_LEN;

enum LoraFlags : uint8_t { F_ACTIVE = 1, F_GPS_FIX = 2, F_IR_ON = 4 };

struct LoraPkt {
  uint8_t  ver = 1;
  uint8_t  flags = 0;
  uint16_t vid = 0;
  uint8_t  prio = 0;
  uint8_t  sats = 0;
  uint32_t counter = 0;
  int32_t  lat_e7 = 0;
  int32_t  lon_e7 = 0;
  uint16_t heading_cdeg = 0;   // 0..35999, 0 = north, clockwise
  uint16_t speed_dms = 0;      // decimetres per second
  uint8_t  hdop_d = 255;       // HDOP * 10
};

size_t lora_encode(const LoraPkt& p, const uint8_t* key, size_t klen, uint8_t out[LORA_PKT_LEN]);
// Returns false if length, magic, version or MAC is wrong.
bool   lora_decode(const uint8_t* in, size_t n, const uint8_t* key, size_t klen, LoraPkt& p);

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------
constexpr double EARTH_R = 6371000.0;
// Local east/north offset (m) of (lat,lon) from origin (lat0,lon0); fine within a few km.
void  enu_offset(double lat0, double lon0, double lat, double lon, double& east, double& north);
float ang_diff_deg(float a, float b);   // signed smallest difference a-b in (-180,180]

}  // namespace evp
