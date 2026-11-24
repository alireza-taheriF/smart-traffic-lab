// EVP-12 intersection decision engine (portable; no Arduino calls).
// Inputs:  decoded IR frames per detector head, decoded+verified LoRa packets.
// Output:  at most ONE active preemption request (approach index) at a time.
#pragma once
#include "evp_proto.h"

namespace evp {

constexpr int MAX_APPROACHES = 4;
constexpr int MAX_TRACKS     = 8;
constexpr int IR_HIST        = 32;

struct Approach {
  const char* name;          // "N", "E", ...
  double stop_lat, stop_lon; // centre of this approach's stop line
  float  inbound_bearing;    // direction vehicles DRIVE when approaching (deg, 0=N, CW)
  float  half_width_m;       // corridor half width
  float  max_dist_m;         // corridor length upstream of the stop line
  int    ir_channel;         // detector head index looking at this approach (-1 = none)
};

struct Config {
  uint8_t  system_code      = 0x5A;
  bool     ir_only_allowed  = true;   // IR alone may trigger (GPS lost / urban canyon)
  bool     gps_only_allowed = true;   // signed GPS alone may trigger early
  float    gps_eta_s        = 25.f;   // GPS-only trigger when ETA <= this
  float    pre_eta_s        = 45.f;   // send PRE (serial only) when ETA <= this
  float    gps_max_hdop     = 2.5f;
  uint8_t  gps_min_sats     = 6;
  float    heading_tol_deg  = 30.f;
  uint32_t ir_confirm_ms    = 400;    // IR+GPS: periodic frames for this long
  uint32_t ir_only_ms       = 1000;   // IR-only: longer confirmation
  uint32_t lora_stale_ms    = 3000;
  uint32_t hold_ms          = 6000;   // keep request this long after last support
  uint32_t max_request_ms   = 120000; // hard cap per request
  uint32_t lockout_ms       = 20000;  // same vehicle cannot re-trigger right away
  float    passed_m         = -12.f;  // s below this = vehicle crossed the stop line
};

enum class Source : uint8_t { NONE, IR_GPS, GPS_ONLY, IR_ONLY };
enum class Event  : uint8_t { REQ_ON, REQ_OFF, PRE, DIR_CONFLICT, REPLAY, BAD_MAC };

struct Request {
  bool     active = false;
  int      approach = -1;
  uint16_t vid = 0;
  uint8_t  prio = 0;
  Source   src = Source::NONE;
  uint32_t t_on_ms = 0, t_support_ms = 0;
  float    eta_s = -1, dist_m = -1;
  uint8_t  ir_level = 0xFF;
};

struct IrStatus {
  bool     ok = false;        // periodic 12 Hz frames present long enough
  uint16_t n_frames = 0;      // frames in last 1 s
  uint16_t n_periodic = 0;    // of those, spaced by k*83.3 ms (+-3 ms)
  float    score = 0;         // weighted by level: HIGH 1, MID 3, LOW 9
  uint8_t  best_level = 0xFF; // weakest level decoded (LOW = closest)
  uint32_t run_ms = 0;        // continuous presence time
  uint16_t rc12 = 0;          // latest rc12
};

typedef void (*EventCb)(Event e, const Request& r, const char* why);

class Engine {
 public:
  void begin(const Config& c, const Approach* a, int n_app, const uint8_t* key, size_t klen, EventCb cb);
  // channel: detector index; t_ms: frame start time
  void on_ir_frame(int channel, const IrFrame& f, uint32_t t_ms);
  // raw LoRa bytes; verifies MAC + replay window. Returns true if accepted.
  bool on_lora(const uint8_t* buf, size_t n, uint32_t t_ms);
  void tick(uint32_t t_ms);            // call every 20..100 ms
  void inhibit(bool on) { inhibit_ = on; }
  // Restore replay protection after a reboot: packets with counter <= c are refused.
  void seed_counter(uint16_t vid, uint32_t c);
  const Request& request() const { return req_; }
  IrStatus ir_status(int channel, uint8_t vid8, uint32_t t_ms, uint32_t need_ms) const;
  int  approach_count() const { return n_app_; }
  const Approach& approach(int i) const { return app_[i]; }

  struct Track {
    bool used = false; bool have = false; uint16_t vid = 0; uint8_t prio = 0; uint8_t flags = 0;
    uint32_t counter = 0, t_ms = 0;
    bool gps_good = false; float s = 0, l = 0, speed = 0, heading = 0, prev_s = 1e9f;
    int approach = -1; uint8_t n_match = 0; float eta = -1;
    uint32_t lockout_until = 0; bool pre_sent = false;
  };
  const Track* track(uint16_t vid) const;

 private:
  struct IrRec { uint32_t t_ms; uint8_t vid8, level; uint16_t rc12; bool used; };
  Config cfg_;
  const Approach* app_ = nullptr; int n_app_ = 0;
  const uint8_t* key_ = nullptr; size_t klen_ = 0;
  EventCb cb_ = nullptr;
  IrRec ir_[MAX_APPROACHES][IR_HIST];
  uint8_t ir_head_[MAX_APPROACHES] = {0};
  Track tr_[MAX_TRACKS];
  Request req_;
  bool inhibit_ = false;
  uint16_t conflict_vid_ = 0; uint32_t conflict_ms_ = 0; bool conflict_any_ = false;
  void conflict(uint16_t vid, int app, uint32_t now);

  Track* find_track(uint16_t vid, bool create);
  void   locate(Track& t, const LoraPkt& p);
  int    best_ir_channel(uint8_t vid8, uint32_t now, uint32_t need_ms, IrStatus* st, bool* ambiguous) const;
  int    channel_to_approach(int ch) const;
  void   emit(Event e, const char* why) { if (cb_) cb_(e, req_, why); }
  void   release(uint32_t now, const char* why);
};

const char* source_name(Source s);

}  // namespace evp
