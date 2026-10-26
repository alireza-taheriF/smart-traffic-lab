// EVP-12 shared protocol implementation. See evp_proto.h.
#include "evp_proto.h"
#include <string.h>
#include <math.h>

namespace evp {

// ------------------------------------------------------------------ CRC / IR
uint8_t crc8_07(const uint8_t* d, size_t n) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < n; ++i) {
    c ^= d[i];
    for (int b = 0; b < 8; ++b) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

uint32_t ir_pack(const IrFrame& f, uint8_t system_code) {
  uint32_t up = ((uint32_t)(f.level & 3) << 22) | ((uint32_t)(f.prio & 3) << 20) |
                ((uint32_t)f.vid8 << 12) | (uint32_t)(f.rc12 & 0x0FFF);
  uint8_t b[3] = {(uint8_t)(up >> 16), (uint8_t)(up >> 8), (uint8_t)up};
  uint8_t crc = crc8_07(b, 3) ^ system_code;
  return (up << 8) | crc;
}

bool ir_unpack(uint32_t w, uint8_t system_code, IrFrame& o) {
  uint32_t up = w >> 8;
  uint8_t b[3] = {(uint8_t)(up >> 16), (uint8_t)(up >> 8), (uint8_t)up};
  if ((uint8_t)(crc8_07(b, 3) ^ system_code) != (uint8_t)(w & 0xFF)) return false;
  o.level = (up >> 22) & 3;
  o.prio  = (up >> 20) & 3;
  o.vid8  = (up >> 12) & 0xFF;
  o.rc12  = up & 0x0FFF;
  return o.level <= LVL_HIGH;
}

size_t ir_build_symbols(uint32_t w, uint16_t* out, size_t max) {
  size_t n = 0;
  auto put = [&](uint32_t us) { if (n < max) out[n++] = (uint16_t)us; };
  put(IR_HDR_MARK_US);
  put(IR_HDR_SPACE_US);
  for (int i = 31; i >= 0; --i) {
    put(IR_T_US);
    put(((w >> i) & 1) ? 3 * IR_T_US : IR_T_US);
  }
  put(IR_T_US);  // stop mark
  return n;      // 67
}

// Tolerances cover TSOP output distortion (-5/f0 .. +6/f0 = -132..+158 us)
// plus optocoupler delay on the field cable.
static inline bool in(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }
static const uint32_t HDR_LO = 1300, HDR_HI = 2400;
static const uint32_t MARK_LO = 120, MARK_HI = 600;
static const uint32_t SP0_LO = 100, SP0_HI = 620;
static const uint32_t SP1_LO = 640, SP1_HI = 1300;

void IrDecoder::reset() { st_ = IDLE; acc_ = 0; nbits_ = 0; }

bool IrDecoder::feed(bool is_mark, uint32_t dur, uint32_t end_us) {
  switch (st_) {
    case IDLE:
      if (is_mark && in(dur, HDR_LO, HDR_HI)) { st_ = HDR_SPACE; start_us_ = end_us - dur; }
      return false;
    case HDR_SPACE:
      if (!is_mark && in(dur, HDR_LO, HDR_HI)) { st_ = BIT_MARK; acc_ = 0; nbits_ = 0; }
      else reset();
      return false;
    case BIT_MARK:
    case STOP_MARK:
      if (is_mark && in(dur, MARK_LO, MARK_HI)) {
        if (st_ == STOP_MARK) { word_ = acc_; reset(); return true; }
        st_ = BIT_SPACE;
        return false;
      }
      reset();
      if (is_mark && in(dur, HDR_LO, HDR_HI)) { st_ = HDR_SPACE; start_us_ = end_us - dur; }
      return false;
    case BIT_SPACE:
      if (!is_mark && in(dur, SP0_LO, SP0_HI))      acc_ = (acc_ << 1);
      else if (!is_mark && in(dur, SP1_LO, SP1_HI)) acc_ = (acc_ << 1) | 1;
      else { reset(); return false; }
      st_ = (++nbits_ == 32) ? STOP_MARK : BIT_MARK;
      return false;
  }
  return false;
}

// ------------------------------------------------------------------ SHA-256
static const uint32_t K[64] = {
  0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
  0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
  0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
  0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
  0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
  0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
  0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
  0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};

static inline uint32_t ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void sha_block(uint32_t h[8], const uint8_t* p) {
  uint32_t w[64];
  for (int i = 0; i < 16; ++i)
    w[i] = ((uint32_t)p[4*i] << 24) | ((uint32_t)p[4*i+1] << 16) | ((uint32_t)p[4*i+2] << 8) | p[4*i+3];
  for (int i = 16; i < 64; ++i) {
    uint32_t s0 = ror(w[i-15], 7) ^ ror(w[i-15], 18) ^ (w[i-15] >> 3);
    uint32_t s1 = ror(w[i-2], 17) ^ ror(w[i-2], 19) ^ (w[i-2] >> 10);
    w[i] = w[i-16] + s0 + w[i-7] + s1;
  }
  uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
  for (int i = 0; i < 64; ++i) {
    uint32_t S1 = ror(e,6) ^ ror(e,11) ^ ror(e,25);
    uint32_t ch = (e & f) ^ (~e & g);
    uint32_t t1 = hh + S1 + ch + K[i] + w[i];
    uint32_t S0 = ror(a,2) ^ ror(a,13) ^ ror(a,22);
    uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
    uint32_t t2 = S0 + mj;
    hh=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
  }
  h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=hh;
}

void Sha256::init() {
  static const uint32_t H0[8] = {0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                                 0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
  memcpy(h, H0, sizeof h); len = 0; blen = 0;
}
void Sha256::update(const uint8_t* d, size_t n) {
  len += n;
  while (n--) { buf[blen++] = *d++; if (blen == 64) { sha_block(h, buf); blen = 0; } }
}
void Sha256::final(uint8_t out[32]) {
  uint64_t bits = len * 8;
  uint8_t pad = 0x80; update(&pad, 1);
  uint8_t z = 0; while (blen != 56) update(&z, 1);
  uint8_t L[8]; for (int i = 0; i < 8; ++i) L[i] = (uint8_t)(bits >> (56 - 8*i));
  update(L, 8);
  for (int i = 0; i < 8; ++i) { out[4*i]=h[i]>>24; out[4*i+1]=h[i]>>16; out[4*i+2]=h[i]>>8; out[4*i+3]=h[i]; }
}

void hmac_sha256(const uint8_t* key, size_t klen, const uint8_t* msg, size_t mlen, uint8_t out[32]) {
  uint8_t k0[64] = {0};
  if (klen > 64) { Sha256 s; s.init(); s.update(key, klen); s.final(k0); }
  else memcpy(k0, key, klen);
  uint8_t ipad[64], opad[64];
  for (int i = 0; i < 64; ++i) { ipad[i] = k0[i] ^ 0x36; opad[i] = k0[i] ^ 0x5c; }
  uint8_t inner[32];
  Sha256 s; s.init(); s.update(ipad, 64); s.update(msg, mlen); s.final(inner);
  s.init(); s.update(opad, 64); s.update(inner, 32); s.final(out);
}

// ------------------------------------------------------------------ LoRa packet
static void w16(uint8_t* p, uint16_t v) { p[0]=v; p[1]=v>>8; }
static void w32(uint8_t* p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static uint16_t r16(const uint8_t* p) { return p[0] | (p[1] << 8); }
static uint32_t r32(const uint8_t* p) { return p[0] | (p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

size_t lora_encode(const LoraPkt& p, const uint8_t* key, size_t klen, uint8_t o[LORA_PKT_LEN]) {
  o[0] = 'E'; o[1] = 'V'; o[2] = p.ver; o[3] = p.flags;
  w16(o+4, p.vid); o[6] = p.prio; o[7] = p.sats;
  w32(o+8, p.counter); w32(o+12, (uint32_t)p.lat_e7); w32(o+16, (uint32_t)p.lon_e7);
  w16(o+20, p.heading_cdeg); w16(o+22, p.speed_dms); o[24] = p.hdop_d; o[25] = 0;
  uint8_t mac[32]; hmac_sha256(key, klen, o, LORA_BODY_LEN, mac);
  memcpy(o + LORA_BODY_LEN, mac, LORA_MAC_LEN);
  return LORA_PKT_LEN;
}

bool lora_decode(const uint8_t* in, size_t n, const uint8_t* key, size_t klen, LoraPkt& p) {
  if (n != LORA_PKT_LEN || in[0] != 'E' || in[1] != 'V' || in[2] != 1) return false;
  uint8_t mac[32]; hmac_sha256(key, klen, in, LORA_BODY_LEN, mac);
  uint8_t diff = 0;  // constant-time compare
  for (size_t i = 0; i < LORA_MAC_LEN; ++i) diff |= mac[i] ^ in[LORA_BODY_LEN + i];
  if (diff) return false;
  p.ver = in[2]; p.flags = in[3]; p.vid = r16(in+4); p.prio = in[6]; p.sats = in[7];
  p.counter = r32(in+8); p.lat_e7 = (int32_t)r32(in+12); p.lon_e7 = (int32_t)r32(in+16);
  p.heading_cdeg = r16(in+20); p.speed_dms = r16(in+22); p.hdop_d = in[24];
  return true;
}

// ------------------------------------------------------------------ geometry
void enu_offset(double lat0, double lon0, double lat, double lon, double& east, double& north) {
  const double d2r = M_PI / 180.0;
  north = (lat - lat0) * d2r * EARTH_R;
  east  = (lon - lon0) * d2r * EARTH_R * cos(lat0 * d2r);
}

float ang_diff_deg(float a, float b) {
  float d = fmodf(a - b, 360.0f);
  if (d > 180.0f) d -= 360.0f;
  if (d <= -180.0f) d += 360.0f;
  return d;
}

}  // namespace evp
