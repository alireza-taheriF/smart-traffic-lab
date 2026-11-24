// Host-side tests for EVP-12 shared code:  g++ -std=c++17 -O2 -I../common host_test.cpp ../common/*.cpp
#include "evp_proto.h"
#include "evp_logic.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <string>
#include <vector>

using namespace evp;
static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); ++fails; } else printf("ok   %s\n", msg); } while (0)

static std::string hex(const uint8_t* d, size_t n) { std::string s; char b[3]; for (size_t i=0;i<n;++i){snprintf(b,3,"%02x",d[i]); s+=b;} return s; }

// ------------------------------------------------------------ crypto
static void test_crypto() {
  uint8_t out[32];
  Sha256 s; s.init(); s.update((const uint8_t*)"abc", 3); s.final(out);
  CHECK(hex(out,32) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "SHA-256('abc')");
  const char* m = "what do ya want for nothing?";
  hmac_sha256((const uint8_t*)"Jefe", 4, (const uint8_t*)m, strlen(m), out);
  CHECK(hex(out,32) == "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", "HMAC-SHA256 RFC4231 case 2");
  // long message (multi-block) against value computed by Python's hmac module (see test/vectors.py)
  std::string big(1000, 'x');
  hmac_sha256((const uint8_t*)"k", 1, (const uint8_t*)big.data(), big.size(), out);
  CHECK(hex(out,32) == "b93e18b28dde167a411967fa971e88a0ef5e6b205a938f7cfb2fa8d45c207d47", "HMAC multi-block matches Python hmac");
}

// ------------------------------------------------------------ IR
static std::mt19937 rng(12345);
static int urand(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

// Feed one frame's symbols through a TSOP-like distortion into a decoder.
static bool push_frame(IrDecoder& d, uint32_t word, uint32_t& t_us, uint32_t& out_word) {
  uint16_t sym[80]; size_t n = ir_build_symbols(word, sym, 80);
  bool got = false;
  int carry = 0;
  for (size_t i = 0; i < n; ++i) {
    bool mark = (i % 2) == 0;
    int dist = mark ? urand(-125, 150) : 0;          // datasheet: -5/f0 .. +6/f0
    int dur = (int)sym[i] + dist - carry;            // spaces absorb the mark stretch
    carry = mark ? dist : 0;
    dur += urand(-20, 20);                           // opto + ISR jitter
    if (dur < 20) dur = 20;
    t_us += dur;
    if (d.feed(mark, (uint32_t)dur, t_us)) { got = true; out_word = d.word(); }
  }
  t_us += 30000; d.feed(false, 30000, t_us);         // inter-frame gap
  return got;
}

static void test_ir() {
  IrFrame f{LVL_MID, 2, 0x34, 0xABC};
  uint32_t w = ir_pack(f, 0x5A); IrFrame g{};
  CHECK(ir_unpack(w, 0x5A, g) && g.level == LVL_MID && g.prio == 2 && g.vid8 == 0x34 && g.rc12 == 0xABC, "IR pack/unpack round trip");
  CHECK(!ir_unpack(w, 0x5B, g), "IR frame rejected with wrong system code");
  uint16_t sym[80]; size_t n = ir_build_symbols(0xFFFFFFFF, sym, 80); uint32_t tot = 0; for (size_t i=0;i<n;++i) tot += sym[i];
  printf("     longest frame = %u us (period %u us), %zu symbols\n", tot, IR_PERIOD_US, n);
  CHECK(tot < IR_PERIOD_US * 6 / 10, "longest frame < 60% of the 83.3 ms period");

  IrDecoder d; d.reset(); uint32_t t = 0; int ok = 0, N = 5000;
  for (int i = 0; i < N; ++i) {
    IrFrame r{(uint8_t)urand(0,2), (uint8_t)urand(0,3), (uint8_t)urand(0,255), (uint16_t)urand(0,4095)};
    uint32_t ww = ir_pack(r, 0x5A), got = 0;
    if (push_frame(d, ww, t, got) && got == ww) ++ok;
  }
  printf("     decoded %d / %d distorted frames\n", ok, N);
  CHECK(ok == N, "decoder survives TSOP pulse-width distortion");

  // Random noise must not create CRC-valid frames.
  d.reset(); int fake = 0; uint32_t tt = 0;
  for (int i = 0; i < 2000000; ++i) {
    bool mark = (i % 2) == 0; uint32_t dur = (uint32_t)urand(50, 3000); tt += dur;
    if (d.feed(mark, dur, tt)) { IrFrame x; if (ir_unpack(d.word(), 0x5A, x)) ++fake; }
  }
  printf("     CRC-valid frames from 2,000,000 random segments: %d\n", fake);
  CHECK(fake <= 1, "noise practically never forms a valid frame");
}

// ------------------------------------------------------------ scenario harness
static const uint8_t KEY[32] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
static const double LAT0 = 36.2970, LON0 = 59.6062;   // a Mashhad intersection (example)
static Approach APP[4] = {
  {"N", LAT0 + 0.00012, LON0, 180.f, 25.f, 600.f, 0},
  {"E", LAT0, LON0 + 0.00015, 270.f, 25.f, 600.f, 1},
  {"S", LAT0 - 0.00012, LON0, 0.f,   25.f, 600.f, 2},
  {"W", LAT0, LON0 - 0.00015, 90.f,  25.f, 600.f, 3}};

struct Log { std::vector<std::string> ev; };
static Log g_log;
static void cb(Event e, const Request& r, const char* why) {
  const char* en[] = {"REQ_ON","REQ_OFF","PRE","DIR_CONFLICT","REPLAY","BAD_MAC"};
  char b[160]; snprintf(b, sizeof b, "%s app=%d vid=%u src=%s eta=%.1f dist=%.0f (%s)", en[(int)e], r.approach, r.vid,
                        source_name(r.src), r.eta_s, r.dist_m, why);
  g_log.ev.push_back(b);
}
static bool has(const char* prefix, int app = -99) {
  for (auto& s : g_log.ev) if (s.rfind(prefix, 0) == 0) {
    if (app == -99) return true;
    char k[16]; snprintf(k, sizeof k, "app=%d ", app); if (s.find(k) != std::string::npos) return true;
  }
  return false;
}

// position on approach a, s metres upstream of its stop line
static void pos_on(const Approach& a, float s, double& lat, double& lon) {
  float b = a.inbound_bearing * (float)M_PI / 180.f;
  double e = -sin(b) * s, n = -cos(b) * s;
  lat = a.stop_lat + n / EARTH_R * 180.0 / M_PI;
  lon = a.stop_lon + e / (EARTH_R * cos(a.stop_lat * M_PI / 180.0)) * 180.0 / M_PI;
}

struct Sim {
  Engine eng; Config cfg; uint32_t counter = 1000; uint16_t vid = 0x0A34;
  bool track_relay_ok = true; int wrong_relay = -1;
  void begin() { g_log.ev.clear(); eng.begin(cfg, APP, 4, KEY, 32, cb); }
  void lora(uint32_t now, double lat, double lon, float hdg, float spd, bool fix = true, uint8_t sats = 9) {
    LoraPkt p; p.flags = F_ACTIVE | (fix ? F_GPS_FIX : 0) | F_IR_ON; p.vid = vid; p.prio = 2; p.sats = sats;
    p.counter = ++counter; p.lat_e7 = (int32_t)llround(lat * 1e7); p.lon_e7 = (int32_t)llround(lon * 1e7);
    p.heading_cdeg = (uint16_t)lroundf(fmodf(hdg + 360.f, 360.f) * 100); p.speed_dms = (uint16_t)lroundf(spd * 10); p.hdop_d = fix ? 9 : 99;
    uint8_t b[LORA_PKT_LEN]; lora_encode(p, KEY, 32, b); eng.on_lora(b, sizeof b, now);
  }
  void ir(uint32_t now, int ch, uint8_t lvl) { IrFrame f{lvl, 2, (uint8_t)(vid & 0xFF), (uint16_t)(counter & 0xFFF)}; eng.on_ir_frame(ch, f, now); }
};

// Which emitter levels decode at distance d on the direct path (link budget, see doc section 3)
static bool decodes(uint8_t lvl, float d, float range_high) {
  float r = lvl == LVL_HIGH ? range_high : lvl == LVL_MID ? range_high * 0.5f : range_high * 0.16f;
  return d <= r;
}
static const uint8_t PATTERN[6] = {LVL_HIGH, LVL_MID, LVL_HIGH, LVL_LOW, LVL_HIGH, LVL_MID};

// Vehicle drives in on approach `ap` from s0 to past the stop line. Options for GPS loss / reflections / wrong IR.
static void drive(Sim& sim, int ap, float s0, float v, bool gps, int reflect_ch, int ir_override_ch, float ir_range, uint32_t& t_end) {
  const Approach& a = APP[ap];
  uint32_t now = 0; float s = s0; int frame = 0; uint32_t next_lora = 0, next_ir = 0;
  int relay_seen_on_other = -1;
  while (s > -60.f) {
    if (now >= next_lora) {
      double la, lo; pos_on(a, s, la, lo);
      double jl = (std::uniform_real_distribution<double>(-3, 3))(rng);   // GPS noise (m)
      la += jl / EARTH_R * 180 / M_PI;
      sim.lora(now, la, lo, a.inbound_bearing + std::uniform_real_distribution<float>(-6, 6)(rng), v, gps, gps ? 9 : 3);
      next_lora += 1000;
    }
    if (now >= next_ir) {
      uint8_t lvl = PATTERN[frame % 6]; frame++;
      int ch = ir_override_ch >= 0 ? ir_override_ch : a.ir_channel;
      if (s > 0 && decodes(lvl, s, ir_range) && urand(0, 99) < 92) sim.ir(now + urand(0, 1), ch, lvl);  // 8% frame loss
      if (reflect_ch >= 0 && s > 0 && s < 120 && lvl == LVL_HIGH && urand(0, 99) < 60) sim.ir(now, reflect_ch, lvl);
      next_ir += 83 + (frame % 3 == 0 ? 1 : 0);
    }
    sim.eng.tick(now);
    const Request& r = sim.eng.request();
    if (r.active && r.approach != ap) relay_seen_on_other = r.approach;
    now += 10; s -= v * 0.010f;
  }
  for (int i = 0; i < 1000; ++i) { sim.eng.tick(now); now += 10; }
  sim.wrong_relay = relay_seen_on_other; t_end = now;
}

static void dump() { for (auto& s : g_log.ev) printf("       | %s\n", s.c_str()); }

static void test_scenarios() {
  uint32_t te;
  { // 1. Normal: from the north, GPS good, reflection into the east head
    Sim sim; sim.begin(); drive(sim, 0, 700, 14, true, 1, -1, 200, te); dump();
    CHECK(has("PRE", 0), "S1: PRE-alert on N from signed GPS");
    CHECK(has("REQ_ON", 0), "S1: request on N");
    CHECK(sim.wrong_relay < 0 && !has("REQ_ON", 1), "S1: east reflection never wins");
    CHECK(has("REQ_OFF", 0), "S1: released after vehicle passed");
  }
  { // 2. GPS lost (urban canyon): IR alone, from the east
    Sim sim; sim.begin(); drive(sim, 1, 400, 12, false, -1, -1, 150, te); dump();
    CHECK(has("REQ_ON", 1) && !has("REQ_ON", 0) && !has("REQ_ON", 2) && !has("REQ_ON", 3), "S2: IR-only request on E");
    CHECK(has("REQ_OFF", 1), "S2: released when IR lost after passing");
  }
  { // 3. Conflict: GPS says S but bound IR arrives on W head (mis-aimed head)
    Sim sim; sim.cfg.gps_only_allowed = false; sim.begin(); drive(sim, 2, 400, 12, true, -1, 3, 200, te); dump();
    CHECK(!has("REQ_ON"), "S3: no request when IR and GPS disagree");
    CHECK(has("DIR_CONFLICT"), "S3: DIR_CONFLICT reported");
  }
  { // 4. Replay + forged MAC rejected
    Sim sim; sim.begin();
    LoraPkt p; p.flags = F_ACTIVE | F_GPS_FIX; p.vid = 7; p.counter = 50; p.sats = 9; p.hdop_d = 8;
    uint8_t b[LORA_PKT_LEN]; lora_encode(p, KEY, 32, b);
    bool a1 = sim.eng.on_lora(b, sizeof b, 0), a2 = sim.eng.on_lora(b, sizeof b, 100);
    b[10] ^= 1; bool a3 = sim.eng.on_lora(b, sizeof b, 200);
    CHECK(a1 && !a2 && !a3 && has("REPLAY") && has("BAD_MAC"), "S4: replayed and tampered packets rejected");
  }
  { // 5. Vehicle on a parallel street 80 m away heading the same way: no request
    Sim sim; sim.begin();
    uint32_t now = 0; const Approach& a = APP[0];
    for (float s = 600; s > -50; s -= 14) {
      double la, lo; pos_on(a, s, la, lo); lo += 80.0 / (EARTH_R * cos(LAT0 * M_PI / 180)) * 180 / M_PI;
      sim.lora(now, la, lo, 180, 14);
      for (int k = 0; k < 100; ++k) { sim.eng.tick(now); now += 10; }
    }
    CHECK(!has("REQ_ON"), "S5: parallel street outside the corridor ignored");
  }
  { // 7. GPS-only disabled: request only once IR on the right head confirms
    Sim sim; sim.cfg.gps_only_allowed = false; sim.begin(); drive(sim, 2, 500, 14, true, 1, -1, 180, te); dump();
    CHECK(has("REQ_ON", 2) && g_log.ev.size() >= 2 && g_log.ev[1].find("src=IR+GPS") != std::string::npos, "S7: IR+GPS confirmation on S");
  }
  { // 6. Inhibit from the traffic controller blocks requests
    Sim sim; sim.begin(); sim.eng.inhibit(true); drive(sim, 3, 500, 14, true, -1, -1, 200, te);
    CHECK(!has("REQ_ON"), "S6: inhibit blocks requests");
  }
}

int main() {
  test_crypto();
  test_ir();
  test_scenarios();
  printf("\n%s (%d failures)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
