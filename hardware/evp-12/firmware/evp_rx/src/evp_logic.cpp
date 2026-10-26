// EVP-12 intersection decision engine. See evp_logic.h.
#include "evp_logic.h"
#include <string.h>
#include <math.h>

namespace evp {

const char* source_name(Source s) {
  switch (s) {
    case Source::IR_GPS:   return "IR+GPS";
    case Source::GPS_ONLY: return "GPS";
    case Source::IR_ONLY:  return "IR";
    default:               return "-";
  }
}

void Engine::begin(const Config& c, const Approach* a, int n, const uint8_t* key, size_t klen, EventCb cb) {
  cfg_ = c; app_ = a; n_app_ = n > MAX_APPROACHES ? MAX_APPROACHES : n;
  key_ = key; klen_ = klen; cb_ = cb;
  memset(ir_, 0, sizeof ir_); memset(ir_head_, 0, sizeof ir_head_);
  for (auto& t : tr_) t = Track();
  req_ = Request();
}

// ------------------------------------------------------------------ IR side
void Engine::on_ir_frame(int ch, const IrFrame& f, uint32_t t) {
  if (ch < 0 || ch >= MAX_APPROACHES) return;
  IrRec& r = ir_[ch][ir_head_[ch]];
  r.t_ms = t; r.vid8 = f.vid8; r.level = f.level; r.rc12 = f.rc12; r.used = true;
  ir_head_[ch] = (ir_head_[ch] + 1) % IR_HIST;
}

IrStatus Engine::ir_status(int ch, uint8_t vid8, uint32_t now, uint32_t need_ms) const {
  IrStatus st;
  if (ch < 0 || ch >= MAX_APPROACHES) return st;
  // Walk oldest -> newest.
  uint32_t prev_t = 0; bool have_prev = false;
  uint32_t run_start = 0, last_t = 0; bool have_last = false;
  for (int k = 0; k < IR_HIST; ++k) {
    const IrRec& r = ir_[ch][(ir_head_[ch] + k) % IR_HIST];
    if (!r.used || r.vid8 != vid8 || now - r.t_ms > 2500) continue;
    // continuous-presence run is measured over the whole history (up to 2.5 s) ...
    if (!have_last || r.t_ms - last_t > 400) run_start = r.t_ms;
    last_t = r.t_ms; have_last = true;
    if (now - r.t_ms > 1000) continue;
    // ... counts, score and periodicity over the last second only
    st.n_frames++;
    st.score += (r.level == LVL_LOW) ? 9.f : (r.level == LVL_MID) ? 3.f : 1.f;
    if (st.best_level == 0xFF || r.level < st.best_level) st.best_level = r.level;
    st.rc12 = r.rc12;
    if (have_prev) {
      float dt = (float)(r.t_ms - prev_t);
      int m = (int)lroundf(dt / 83.333f);
      if (m >= 1 && m <= 4 && fabsf(dt - m * 83.333f) <= 3.0f) st.n_periodic++;
    }
    prev_t = r.t_ms; have_prev = true;
  }
  if (have_last && now - last_t <= 300) st.run_ms = last_t - run_start;
  st.ok = st.run_ms >= need_ms && st.n_periodic >= 3;
  return st;
}

int Engine::best_ir_channel(uint8_t vid8, uint32_t now, uint32_t need, IrStatus* out, bool* amb) const {
  int best = -1; float bs = 0, second = 0; IrStatus bst;
  for (int ch = 0; ch < MAX_APPROACHES; ++ch) {
    IrStatus s = ir_status(ch, vid8, now, need);
    if (!s.ok) continue;
    if (s.score > bs) { second = bs; bs = s.score; best = ch; bst = s; }
    else if (s.score > second) second = s.score;
  }
  if (out) *out = bst;
  // A reflection usually decodes only HIGH frames, the direct path decodes MID/LOW too.
  if (amb) *amb = best >= 0 && second >= 0.6f * bs;
  return best;
}

int Engine::channel_to_approach(int ch) const {
  for (int i = 0; i < n_app_; ++i) if (app_[i].ir_channel == ch) return i;
  return -1;
}

// ------------------------------------------------------------------ LoRa side
Engine::Track* Engine::find_track(uint16_t vid, bool create) {
  Track* freeslot = nullptr; Track* oldest = &tr_[0];
  for (auto& t : tr_) {
    if (t.used && t.vid == vid) return &t;
    if (!t.used && !freeslot) freeslot = &t;
    if (t.t_ms < oldest->t_ms) oldest = &t;
  }
  if (!create) return nullptr;
  Track* t = freeslot ? freeslot : oldest;
  *t = Track(); t->used = true; t->vid = vid;
  return t;
}

void Engine::seed_counter(uint16_t vid, uint32_t c) {
  Track* t = find_track(vid, true);
  t->have = true; if (c > t->counter) t->counter = c; t->t_ms = 0; t->flags = 0;
}

const Engine::Track* Engine::track(uint16_t vid) const {
  for (auto& t : tr_) if (t.used && t.vid == vid) return &t;
  return nullptr;
}

void Engine::locate(Track& t, const LoraPkt& p) {
  t.speed = p.speed_dms / 10.f;
  t.heading = p.heading_cdeg / 100.f;
  t.gps_good = (p.flags & F_GPS_FIX) && p.sats >= cfg_.gps_min_sats && p.hdop_d <= cfg_.gps_max_hdop * 10.f;
  double lat = p.lat_e7 / 1e7, lon = p.lon_e7 / 1e7;
  int found = -1; float fs = 0, fl = 0;
  for (int i = 0; i < n_app_ && t.gps_good; ++i) {
    double e, n; enu_offset(app_[i].stop_lat, app_[i].stop_lon, lat, lon, e, n);
    float b = app_[i].inbound_bearing * (float)M_PI / 180.f;
    float ux = -sinf(b), uy = -cosf(b);              // unit vector pointing upstream
    float s = (float)(e * ux + n * uy);              // distance upstream of stop line
    float l = (float)fabs(e * uy - n * ux);          // lateral offset
    bool heading_ok = t.speed < 2.f || fabsf(ang_diff_deg(t.heading, app_[i].inbound_bearing)) <= cfg_.heading_tol_deg;
    if (s >= cfg_.passed_m && s <= app_[i].max_dist_m && l <= app_[i].half_width_m && heading_ok) {
      if (found < 0 || s < fs) { found = i; fs = s; fl = l; }
    }
  }
  if (found >= 0) {
    t.n_match = (t.approach == found) ? (uint8_t)(t.n_match < 250 ? t.n_match + 1 : 250) : 1;
    t.prev_s = (t.approach == found) ? t.s : 1e9f;
    t.approach = found; t.s = fs; t.l = fl;
    t.eta = fs > 0 ? fs / (t.speed > 2.f ? t.speed : 2.f) : 0.f;
  } else {
    // Left the corridor: if it was close to the stop line, it has crossed it.
    if (t.approach >= 0 && t.s < 40.f) t.s = cfg_.passed_m - 1.f;
    t.approach = -1; t.n_match = 0; t.eta = -1;
  }
}

bool Engine::on_lora(const uint8_t* buf, size_t n, uint32_t now) {
  LoraPkt p;
  if (!lora_decode(buf, n, key_, klen_, p)) { Request r; if (cb_) cb_(Event::BAD_MAC, r, "mac/format"); return false; }
  Track* t = find_track(p.vid, true);
  if (t->have && p.counter <= t->counter) {
    Request r; r.vid = p.vid; if (cb_) cb_(Event::REPLAY, r, "counter not increasing"); return false;
  }
  t->have = true; t->counter = p.counter; t->t_ms = now; t->prio = p.prio; t->flags = p.flags;
  locate(*t, p);
  return true;
}

// ------------------------------------------------------------------ decisions
void Engine::release(uint32_t now, const char* why) {
  if (!req_.active) return;
  Track* t = find_track(req_.vid, false);
  if (t) t->lockout_until = now + cfg_.lockout_ms;
  req_.active = false;
  emit(Event::REQ_OFF, why);
  req_ = Request();
}

void Engine::conflict(uint16_t vid, int app, uint32_t now) {
  // report at most once per 5 s per vehicle
  if (conflict_any_ && conflict_vid_ == vid && now - conflict_ms_ < 5000) return;
  conflict_any_ = true; conflict_vid_ = vid; conflict_ms_ = now;
  Request r; r.vid = vid; r.approach = app;
  if (cb_) cb_(Event::DIR_CONFLICT, r, "IR and GPS disagree");
}

void Engine::tick(uint32_t now) {
  struct Cand { bool ok; int app; uint16_t vid; uint8_t prio; Source src; float eta, dist; uint8_t lvl; };
  Cand best{false, -1, 0, 0, Source::NONE, -1, -1, 0xFF};
  bool owner_supported = false;

  // Vehicle ids to evaluate: LoRa tracks + vid8 values seen on IR in the last second.
  uint16_t ids[MAX_TRACKS + MAX_APPROACHES * IR_HIST]; bool from_ir[MAX_TRACKS + MAX_APPROACHES * IR_HIST];
  int nid = 0;
  for (auto& t : tr_) if (t.used && now - t.t_ms <= cfg_.lora_stale_ms) { ids[nid] = t.vid; from_ir[nid++] = false; }
  for (int ch = 0; ch < MAX_APPROACHES; ++ch)
    for (int k = 0; k < IR_HIST; ++k) {
      const IrRec& r = ir_[ch][k];
      if (!r.used || now - r.t_ms > 1000) continue;
      bool known = false;
      for (int j = 0; j < nid; ++j) if ((ids[j] & 0xFF) == r.vid8) { known = true; break; }
      if (!known) { ids[nid] = r.vid8; from_ir[nid++] = true; }
    }

  for (int i = 0; i < nid; ++i) {
    Track* t = from_ir[i] ? nullptr : find_track(ids[i], false);
    uint8_t vid8 = ids[i] & 0xFF;
    bool lora_fresh = t && now - t->t_ms <= cfg_.lora_stale_ms && (t->flags & F_ACTIVE);
    int lora_app = (lora_fresh && t->gps_good) ? t->approach : -1;

    // IR evidence (bound to this vehicle's signed counter when we have one)
    IrStatus st; bool amb = false;
    int ch = best_ir_channel(vid8, now, cfg_.ir_confirm_ms, &st, &amb);
    bool ir_bound = false;
    if (ch >= 0 && t) {
      uint16_t d = (uint16_t)(((t->counter & 0x0FFF) - st.rc12) & 0x0FFF);
      ir_bound = d <= 3;
    }
    int ir_app = ch >= 0 ? channel_to_approach(ch) : -1;

    Cand c{false, -1, ids[i], t ? t->prio : (uint8_t)0, Source::NONE, -1, -1, st.best_level};
    if (lora_app >= 0 && ir_app >= 0 && ir_bound) {
      if (lora_app == ir_app) { c.ok = true; c.app = lora_app; c.src = Source::IR_GPS; }
      else { conflict(ids[i], ir_app, now); continue; }
    } else if (lora_app >= 0 && cfg_.gps_only_allowed && t->n_match >= 2 && t->s < t->prev_s + 1.f && t->eta <= cfg_.gps_eta_s) {
      // A GPS-only request is refused if IR sees this vehicle on another approach.
      if (ir_app >= 0 && ir_app != lora_app && ir_bound) { conflict(ids[i], ir_app, now); continue; }
      c.ok = true; c.app = lora_app; c.src = Source::GPS_ONLY;
    } else if (ir_app >= 0 && cfg_.ir_only_allowed && !amb) {
      // IR alone: only when GPS cannot place the vehicle (no fresh signed fix).
      bool gps_says_elsewhere = lora_fresh && t->gps_good;
      IrStatus st2 = ir_status(ch, vid8, now, cfg_.ir_only_ms);
      if (!gps_says_elsewhere && st2.ok) { c.ok = true; c.app = ir_app; c.src = Source::IR_ONLY; }
    }
    if (t && lora_app >= 0) { c.eta = t->eta; c.dist = t->s; c.prio = t->prio; }

    // PRE-alert (serial only) for a signed vehicle approaching inside a corridor.
    if (t && lora_app >= 0 && t->eta <= cfg_.pre_eta_s && !t->pre_sent && !(req_.active && req_.vid == t->vid)) {
      Request r; r.vid = t->vid; r.approach = lora_app; r.eta_s = t->eta; r.dist_m = t->s; r.prio = t->prio;
      t->pre_sent = true; if (cb_) cb_(Event::PRE, r, "signed vehicle in corridor");
    }
    if (t && lora_app < 0) t->pre_sent = false;

    // Support for the current owner
    if (req_.active && (req_.vid & 0xFF) == vid8 && c.ok && c.app == req_.approach) {
      owner_supported = true; req_.t_support_ms = now; req_.src = c.src;
      req_.eta_s = c.eta; req_.dist_m = c.dist; req_.ir_level = c.lvl;
    }
    if (!c.ok) continue;
    if (t && now < t->lockout_until) continue;
    auto key = [](float e) { return e < 0 ? 1e9f : e; };
    if (!best.ok || c.prio > best.prio || (c.prio == best.prio && key(c.eta) < key(best.eta))) best = c;
  }

  // Owner left the corridor having crossed the stop line -> release at once.
  if (req_.active) {
    Track* ot = find_track(req_.vid, false);
    if (ot && now - ot->t_ms <= cfg_.lora_stale_ms && ot->gps_good && ot->s < cfg_.passed_m) release(now, "vehicle passed");
  }
  if (req_.active) {
    if (inhibit_)                                         release(now, "inhibited");
    else if (now - req_.t_on_ms > cfg_.max_request_ms)    release(now, "max time");
    else if (!owner_supported && now - req_.t_support_ms > cfg_.hold_ms) release(now, "lost");
    return;  // first come, first served: never switch approach while active
  }
  if (inhibit_ || !best.ok) return;
  req_.active = true; req_.approach = best.app; req_.vid = best.vid; req_.prio = best.prio;
  req_.src = best.src; req_.t_on_ms = now; req_.t_support_ms = now;
  req_.eta_s = best.eta; req_.dist_m = best.dist; req_.ir_level = best.lvl;
  emit(Event::REQ_ON, source_name(best.src));
}

}  // namespace evp
