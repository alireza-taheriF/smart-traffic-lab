/* traffic-core.js — simulation core for the traffic-signal comparison lab.
   Two independent simulators (fixed-time "current" vs. "smart") are driven by ONE shared demand stream,
   so both see the identical vehicles at the identical times.
   Modules mirror the architecture document: Perception → Decider → Safety kernel → Signal, + EV priority + Watchdog. */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.TrafficLab = factory();
})(this, function () {
  'use strict';

  /* ---------- constants (meters, seconds) ---------- */
  var R = 13, LW = 3.3, LANES = 2, SPACING = 180, ENTRY = 125;
  var V_LINK = 13.9;           // 50 km/h
  var YELLOW = 3, ALLRED = 2;
  var MIN_GREEN = 12, MIN_GREEN_EV = 5, MAX_GREEN = 55, MAX_WAIT = 130, EV_HOLD_MAX = 45;
  var B = 4.2, TAU = 1.0, MINGAP = 1.8;
  var GPS_RANGE = 230, VIS_RANGE = 110;
  var SIDES = [[0, -1], [1, 0], [0, 1], [-1, 0]];
  var SIDE_FA = ['شمال', 'شرق', 'جنوب', 'غرب'];
  var LEN = { car: 4.6, moto: 2.2, amb: 6.0, fire: 9.5, police: 4.8 };
  var ACC = { car: 2.6, moto: 3.6, amb: 3.4, fire: 2.2, police: 3.8 };
  var VF = { car: 1.0, moto: 1.08, amb: 1.15, fire: 1.05, police: 1.15 };
  var EVRANK = { amb: 3, fire: 3, police: 2 };
  var KIND_FA = { car: 'خودرو', moto: 'موتور', amb: 'آمبولانس', fire: 'آتش‌نشانی', police: 'پلیس' };
  var CRAWL = 6;

  var TUNE = { ratio: 2.2, off: 4, minT: 13, minL: 10, leftThr: 2.7, gap: 2, maxL: 23, maxT: 68, waitDiv: 25, dynAR: 1 };
  var opp = function (s) { return (s + 2) % 4; };
  var right = function (dx, dy) { return [-dy, dx]; };

  function mulberry32(a) {
    return function () {
      a |= 0; a = a + 0x6D2B79F5 | 0;
      var t = Math.imul(a ^ a >>> 15, 1 | a);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
  }

  /* ---------- scenarios ---------- */
  var SCENARIOS = {
    cross: { name: 'چهارراه', nodes: [[0, 0]] },
    tee: { name: 'سه‌راه', nodes: [[0, 0]], missing: { '0,0': [0] } },
    corridor: { name: 'کریدور ۳ چهارراه', nodes: [[0, 0], [1, 0], [2, 0]] },
    grid: { name: 'شبکه ۲×۲', nodes: [[0, 0], [1, 0], [0, 1], [1, 1]] }
  };

  /* ---------- network builder ---------- */
  function lanePt(n, s, inbound, far, lane) {
    var h = inbound ? [-SIDES[s][0], -SIDES[s][1]] : SIDES[s];
    var r = right(h[0], h[1]);
    var d = R + (far ? ENTRY : 0), lat = (lane + 0.5) * LW;
    return { x: n.x + SIDES[s][0] * d + r[0] * lat, y: n.y + SIDES[s][1] * d + r[1] * lat };
  }


  function buildNetwork(key, prot) {
    var sc = SCENARIOS[key];
    var NL = prot ? 3 : 2;
    // protected: lane0 = left only, lane1 = through, lane2 = through+right; permissive: lane0 = left+through, lane1 = through+right
    var LANE_TURNS = prot ? [[3], [0], [0, 1]] : [[3, 0], [0, 1]];
    var nodes = [], links = [], conns = [], entries = [], byPos = {};
    sc.nodes.forEach(function (p, i) {
      var n = { id: i, gx: p[0], gy: p[1], x: p[0] * SPACING, y: p[1] * SPACING, sides: [true, true, true, true],
        inL: [null, null, null, null], outL: [null, null, null, null], conn: [0, 1, 2, 3].map(function () { return [[], [], []]; }), connList: [],
        phases: [], phaseOf: [-1, -1, -1, -1], sig: null };
      var miss = (sc.missing && sc.missing[p.join(',')]) || [];
      miss.forEach(function (s) { n.sides[s] = false; });
      nodes.push(n); byPos[p.join(',')] = n;
    });
    function mkLink(kind, A, Bp, props) {
      var dx = Bp.x - A.x, dy = Bp.y - A.y, len = Math.sqrt(dx * dx + dy * dy);
      var L = { id: links.length, kind: kind, ax: A.x, ay: A.y, bx: Bp.x, by: Bp.y, len: len, dx: dx / len, dy: dy / len,
        vlim: V_LINK, toNode: null, toSide: -1, fromNode: null, fromSide: -1, vehicles: [] };
      for (var k in props) L[k] = props[k];
      links.push(L); return L;
    }
    nodes.forEach(function (n) {
      for (var s = 0; s < 4; s++) {
        if (!n.sides[s]) continue;
        var m = byPos[(n.gx + SIDES[s][0]) + ',' + (n.gy + SIDES[s][1])];
        if (m && m.sides[opp(s)]) {
          // internal directed link n(out side s) -> m(in side opp(s))
          n.outL[s] = []; m.inL[opp(s)] = [];
          for (var ln = 0; ln < NL; ln++) {
            var L = mkLink('internal', lanePt(n, s, false, false, ln), lanePt(m, opp(s), true, false, ln),
              { fromNode: n, fromSide: s, toNode: m, toSide: opp(s), lane: ln });
            n.outL[s].push(L); m.inL[opp(s)].push(L);
          }
        } else if (!m) {
          var A = { weight: (s % 2 === 0) ? 0.65 : 1.0, toNode: n, toSide: s, lanes: [], idx: entries.length,
            label: SIDE_FA[s] + (sc.nodes.length > 1 ? ' (چهارراه ' + (n.id + 1) + ')' : '') };
          entries.push(A); n.inL[s] = []; n.outL[s] = [];
          for (var ln2 = 0; ln2 < NL; ln2++) {
            var E = mkLink('entry', lanePt(n, s, true, true, ln2), lanePt(n, s, true, false, ln2), { toNode: n, toSide: s, lane: ln2, approach: A });
            A.lanes.push(E); n.inL[s].push(E);
            n.outL[s].push(mkLink('exit', lanePt(n, s, false, false, ln2), lanePt(n, s, false, true, ln2), { fromNode: n, fromSide: s, lane: ln2 }));
          }
        }
      }
    });
    // connectors
    nodes.forEach(function (n) {
      for (var a = 0; a < 4; a++) {
        if (!n.sides[a]) continue;
        for (var b = 0; b < 4; b++) {
          if (!n.sides[b] || b === a) continue;
          var h = opp(a), turn = (b - h + 4) % 4;
          for (var lane = 0; lane < NL; lane++) {
          if (LANE_TURNS[lane].indexOf(turn) < 0) continue;
          var P0 = lanePt(n, a, true, false, lane), P2 = lanePt(n, b, false, false, lane), P1;
          if (turn === 0) P1 = { x: (P0.x + P2.x) / 2, y: (P0.y + P2.y) / 2 };
          else if (SIDES[h][0] !== 0) P1 = { x: P2.x, y: P0.y };
          else P1 = { x: P0.x, y: P2.y };
          var pts = [], cum = [0], N = 16;
          for (var i = 0; i <= N; i++) {
            var t = i / N, u = 1 - t;
            pts.push({ x: u * u * P0.x + 2 * u * t * P1.x + t * t * P2.x, y: u * u * P0.y + 2 * u * t * P1.y + t * t * P2.y });
            if (i > 0) cum.push(cum[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y));
          }
          var C = { id: conns.length, kind: 'conn', node: n, a: a, b: b, turn: turn, pts: pts, cum: cum, len: cum[N],
            vehicles: [], conflicts: [], vlim: turn === 0 ? 10 : (turn === 1 ? 5.5 : 6.5), inLink: n.inL[a][lane], outLink: n.outL[b][lane], lane: lane };
          conns.push(C); n.conn[a][lane][b] = C; n.connList.push(C);
          }
        }
      }
      n.connList.forEach(function (c1) {
        n.connList.forEach(function (c2) {
          if (c1 === c2 || c1.a === c2.a) return;
          var conflict = false;
          if (c1.b === c2.b && c1.lane === c2.lane) conflict = true;
          else {
            for (var i = 0; i < c1.pts.length && !conflict; i++)
              for (var j = 0; j < c2.pts.length; j++)
                if (Math.hypot(c1.pts[i].x - c2.pts[j].x, c1.pts[i].y - c2.pts[j].y) < 2.7) { conflict = true; break; }
          }
          if (conflict) c1.conflicts.push(c2);
        });
      });
      n.protected = !!prot;
      [[0, 2], [1, 3]].forEach(function (axis) {
        var sides = axis.filter(function (s) { return n.sides[s]; });
        if (!sides.length) return;
        var nm = sides.map(function (s) { return SIDE_FA[s]; }).join(' و ');
        var hasLeft = sides.some(function (s) { return n.sides[(opp(s) + 3) % 4]; });
        var noThrough = sides.every(function (s) { return !n.sides[opp(s)]; });   // T-junction stem: left+right never conflict -> one phase
        if (prot && noThrough) n.phases.push({ sides: sides, turns: [0, 1, 3], kind: 'A', name: nm + ' · چپ و راست' });
        else if (prot) {
          n.phases.push({ sides: sides, turns: [0, 1], kind: 'T', name: nm + ' · مستقیم و راست' });
          if (hasLeft) n.phases.push({ sides: sides, turns: [3], kind: 'L', name: nm + ' · فقط چپ' });
        } else n.phases.push({ sides: sides, turns: [0, 1, 3], kind: 'A', name: nm });
      });
      n.phases.forEach(function (ph, pi) { if (ph.kind !== 'L') ph.sides.forEach(function (s) { n.phaseOf[s] = pi; }); });
      n.phaseFor = function (side, turn) {
        for (var i = 0; i < n.phases.length; i++) { var ph = n.phases[i]; if (ph.sides.indexOf(side) >= 0 && ph.turns.indexOf(turn) >= 0) return i; }
        return n.phaseOf[side];
      };
    });
    return { key: key, nodes: nodes, links: links, conns: conns, entries: entries, lanes: NL, protected: !!prot };
  }

  /* ---------- demand (shared) ---------- */
  function Demand(net, seed) {
    this.rate = 800; this.surge = null; this.vid = 1; this.evRng = mulberry32(seed * 31 + 5);
    this.entries = net.entries.map(function (e, i) {
      return { idx: i, weight: e.weight, rng: mulberry32(seed * 977 + i * 131 + 7), next: 0 };
    });
    var self = this;
    this.entries.forEach(function (e) { e.next = self.draw(e, 0); });
  }
  Demand.prototype.rateOf = function (e, t) {
    var r = this.rate * e.weight;
    if (this.surge && this.surge.entry === e.idx && t < this.surge.until) r *= 2.6;
    return r;
  };
  Demand.prototype.draw = function (e, t) {
    var lam = Math.max(this.rateOf(e, t), 5) / 3600;
    return t + (-Math.log(1 - e.rng()) / lam);
  };
  Demand.prototype.setRate = function (r, t) {
    this.rate = r; var self = this;
    this.entries.forEach(function (e) { e.next = self.draw(e, t); });
  };
  Demand.prototype.makeSpec = function (rng, entry, kind) {
    var u = rng();
    var k = kind || (u < 0.13 ? 'moto' : 'car');
    var turns = [];
    for (var i = 0; i < 8; i++) { var q = rng(); turns.push(q < 0.64 ? 0 : (q < 0.84 ? 1 : 3)); }
    var gps = rng() < 0.6;
    return { id: this.vid++, kind: k, len: LEN[k], turns: turns, gps: gps, entry: entry };
  };
  Demand.prototype.step = function (t, dt) {
    var out = [], self = this;
    this.entries.forEach(function (e) {
      var guard = 0;
      while (e.next <= t && guard++ < 20) {
        out.push(self.makeSpec(e.rng, e.idx, null));
        e.next = self.draw(e, e.next);
      }
    });
    return out;
  };
  Demand.prototype.makeEV = function (kind, entryIdx) {
    var idx = entryIdx == null ? Math.floor(this.evRng() * this.entries.length) : entryIdx;
    var s = this.makeSpec(this.evRng, idx, kind);
    s.gps = this.evRng() < 0.6;
    return s;
  };
  Demand.prototype.triggerSurge = function (t) {
    var idx = Math.floor(this.evRng() * this.entries.length);
    this.surge = { entry: idx, until: t + 90 };
    var e = this.entries[idx]; e.next = this.draw(e, t);
    return idx;
  };

  /* ---------- vehicle ---------- */
  function Veh(spec, t) {
    this.id = spec.id; this.spec = spec; this.kind = spec.kind; this.len = spec.len;
    this.isEV = EVRANK[spec.kind] !== undefined; this.vf = VF[spec.kind]; this.acc = ACC[spec.kind];
    this.link = null; this.s = 0; this.v = 0; this.hop = 0; this.nextConn = null;
    this.t0 = t; this.stopped = 0; this.stops = 0; this.wasStopped = false; this.curWait = 0;
    this.redWait = 0; this.risky = false; this.riskCounted = false; this.vcap = null;
    this.latOff = 0; this.visT = 0; this.audT = 0; this.detected = null; this.spawned = false;
  }

  /* ---------- simulator ---------- */
  function Sim(key, mode, opts) {
    this.key = key; this.mode = mode; this.net = buildNetwork(key, opts.leftMode === 'protected');
    this.opts = opts;
    this.t = 0; this.acc1 = 0;
    this.rng = mulberry32(4242 + (mode === 'smart' ? 17 : 0));
    var NL = this.net.lanes; this.pending = this.net.entries.map(function () { var a = []; for (var i = 0; i < NL; i++) a.push([]); return a; });
    this.vehs = [];
    this.stats = { n: 0, sumStop: 0, sumTravel: 0, sumStops: 0, redRuns: 0, rejected: 0, risky: 0, ev: [], recent: [], preempts: 0 };
    this.log = []; this.evLog = []; this.lastLogAt = {};
    var self = this;
    this.net.nodes.forEach(function (n) {
      var off = (n.id * 11) % 25;
      n.sig = { phase: (n.id % 2) % n.phases.length, state: 'G', t: 0, greenElapsed: off, target: null, hold: false, holdT: 0 };
      n.pv = [null, null, null, null]; n.conf = 1; n.confBad = 0; n.confOk = 0; n.fallback = false;
      n.pre = null; n.preRel = 0; n.lastPressure = [0, 0];
    });
  }

  Sim.prototype.resetStats = function () {
    this.stats = { n: 0, sumStop: 0, sumTravel: 0, sumStops: 0, redRuns: 0, rejected: 0, risky: 0, ev: [], recent: [], preempts: 0 };
    this.log = []; this.evLog = [];
    this.vehs.forEach(function (v) { v.stopped = 0; v.stops = 0; });
    this.pending.forEach(function (p) { p.forEach(function (q) { q.forEach(function (v) { v.stopped = 0; }); }); });
  };

  Sim.prototype.say = function (text, kind, key, gap) {
    if (key) { var last = this.lastLogAt[key]; if (last != null && this.t - last < (gap || 8)) return; this.lastLogAt[key] = this.t; }
    this.log.unshift({ t: this.t, text: text, kind: kind || 'info' });
    if (this.log.length > 80) this.log.pop();
  };
  Sim.prototype.evSay = function (text, kind) {
    this.evLog.unshift({ t: this.t, text: text, kind: kind || 'info' });
    if (this.evLog.length > 60) this.evLog.pop();
  };

  Sim.prototype.light = function (node, side, turn) {
    var sg = node.sig;
    if (sg.state === 'AR') return 'R';
    var ph = node.phases[sg.phase];
    if (ph.sides.indexOf(side) >= 0 && ph.turns.indexOf(turn || 0) >= 0) return sg.state === 'Y' ? 'Y' : 'G';
    return 'R';
  };
  // seconds until the (side, turn) movement shows green under the fixed-time plan (the Iran-style countdown)
  Sim.prototype.countdown = function (node, side, turn) {
    var sg = node.sig, np = node.phases.length, o = this.opts;
    var dur = function (i) { return node.phases[i].kind === 'L' ? o.leftFixed : o.greenFixed; };
    var tgt = node.phaseFor(side, turn);
    if (sg.phase === tgt && sg.state === 'G') return Math.max(0, dur(tgt) - sg.greenElapsed);
    var t = 0, nxt;
    if (sg.state === 'G') { t += Math.max(0, dur(sg.phase) - sg.greenElapsed) + YELLOW + ALLRED; nxt = (sg.phase + 1) % np; }
    else if (sg.state === 'Y') { t += (YELLOW - sg.t) + ALLRED; nxt = sg.target != null ? sg.target : (sg.phase + 1) % np; }
    else { t += Math.max(0, ALLRED - sg.t); nxt = sg.target != null ? sg.target : (sg.phase + 1) % np; }
    var guard = 0;
    while (nxt !== tgt && guard++ < 8) { t += dur(nxt) + YELLOW + ALLRED; nxt = (nxt + 1) % np; }
    return t;
  };

  /* --- safety kernel: the ONLY door to the signal --- */
  Sim.prototype.request = function (node, target, reason, emerg, force) {
    var sg = node.sig;
    if (sg.state !== 'G') {
      if (emerg && sg.target !== target) { sg.target = target; }
      return false;
    }
    if (target === sg.phase) return false;
    var minG = force ? 0 : (emerg ? MIN_GREEN_EV : (node.phases[sg.phase].kind === 'L' ? TUNE.minL : TUNE.minT));
    if (sg.greenElapsed < minG) {
      if (this.t - (this.lastLogAt['rejc' + node.id] || -99) > 6) { this.stats.rejected++; this.lastLogAt['rejc' + node.id] = this.t; }
      this.say('هسته‌ی ایمنی درخواست تغییر فاز را رد کرد (چهارراه ' + (node.id + 1) + '): حداقل سبز هنوز تمام نشده', 'kernel', 'rej' + node.id, 12);
      return false;
    }
    sg.state = 'Y'; sg.t = 0; sg.target = target; sg.hold = false;
    return true;
  };
  Sim.prototype.advanceSignal = function (node, dt) {
    var sg = node.sig; sg.t += dt;
    if (sg.state === 'G') sg.greenElapsed += dt;
    else if (sg.state === 'Y' && sg.t >= YELLOW) { sg.state = 'AR'; sg.t = 0; }
    else if (sg.state === 'AR' && (sg.t >= ALLRED || (this.mode === 'smart' && TUNE.dynAR && sg.t >= 1.0 && node.connList.every(function (c) { return !c.vehicles.length; })))) {
      sg.state = 'G'; sg.phase = sg.target != null ? sg.target : (sg.phase + 1) % node.phases.length;
      sg.target = null; sg.t = 0; sg.greenElapsed = 0; sg.holdT = 0;
    }
  };

  /* --- controllers (decide once per second) --- */
  Sim.prototype.fixedCtl = function (node, green, leftG) {
    var sg = node.sig; if (sg.state !== 'G') return;
    var dur = node.phases[sg.phase].kind === 'L' ? leftG : green;
    if (sg.greenElapsed >= dur) this.request(node, (sg.phase + 1) % node.phases.length, 'fixed', false, true);
  };

  Sim.prototype.perceive = function (node) {
    var health = this.opts.fault ? 0.12 : 0.96, self = this;
    node.conf = this.opts.fault ? 0.1 + 0.1 * this.rng() : 0.93 + 0.06 * this.rng();
    for (var s = 0; s < 4; s++) {
      var Ls = node.inL[s]; if (!Ls) { node.pv[s] = null; continue; }
      var q = 0, far = 0, moving = 0, stoppedN = 0, maxWait = 0, near = 0, g = { L: { q: 0, near: 0, w: 0 }, T: { q: 0, near: 0, w: 0 } };
      Ls.forEach(function (L) { var grp = (node.protected && L.lane === 0) ? g.L : g.T; L.vehicles.forEach(function (v) {
        if (v.curWait > maxWait) maxWait = v.curWait;
        if (v.curWait > grp.w) grp.w = v.curWait;
        if (self.rng() > health) return;
        var d = L.len - v.s, add = 0;
        if (d <= 130) { if (d <= 60) { near++; grp.near++; } if (v.v < 1.0) { add = 1; stoppedN++; } else { add = 0.6; moving++; } q += add; grp.q += add; }
        else if (self.opts.coopt && L.kind === 'internal') { far += 0.3; g.T.q += 0.3; }
      }); });
      var pen = 0;
      var through = node.outL[opp(s)];
      if (through && through[0].kind === 'internal') {
        var cnt = 0; through.forEach(function (T) { cnt += T.vehicles.length; });
        var load = cnt / (through.length * through[0].len / 7.6);
        pen = 6 * Math.max(0, load - 0.7);
      }
      node.pv[s] = { q: q, far: far, moving: moving, stopped: stoppedN, pressure: q + far - pen + maxWait / TUNE.waitDiv, near: near, maxWait: maxWait, pen: pen,
        pL: g.L.q + g.L.w / TUNE.waitDiv, pT: g.T.q - pen + g.T.w / TUNE.waitDiv, nearL: g.L.near, nearT: g.T.near, wL: g.L.w, wT: g.T.w };
    }
  };

  Sim.prototype.phasePressure = function (node, p) {
    var ph = node.phases[p], tot = 0; ph.sides.forEach(function (s) { var v = node.pv[s]; if (v) tot += ph.kind === 'L' ? v.pL : (ph.kind === 'T' ? v.pT : v.pressure); }); return tot;
  };
  Sim.prototype.phaseNear = function (node, p) {
    var ph = node.phases[p], t = 0; ph.sides.forEach(function (s) { var v = node.pv[s]; if (v) t += ph.kind === 'L' ? v.nearL : (ph.kind === 'T' ? v.nearT : v.near); }); return t;
  };
  Sim.prototype.phaseWait = function (node, p) {
    var ph = node.phases[p], m = 0; ph.sides.forEach(function (s) { var v = node.pv[s]; if (!v) return; var w = ph.kind === 'L' ? v.wL : (ph.kind === 'T' ? v.wT : v.maxWait); if (w > m) m = w; }); return m;
  };

  Sim.prototype.watchdog = function (node) {
    if (node.conf < 0.5) { node.confBad++; node.confOk = 0; } else { node.confOk++; node.confBad = 0; }
    if (!node.fallback && node.confBad >= 3) {
      node.fallback = true;
      this.say('ناظر: اطمینان حسگر پایین است (چهارراه ' + (node.id + 1) + ') ← برگشت به برنامه‌ی ثابت فعلی', 'watchdog');
    } else if (node.fallback && node.confOk >= 10) {
      node.fallback = false;
      this.say('ناظر: حسگر سالم شد (چهارراه ' + (node.id + 1) + ') ← کنترل تطبیقی دوباره فعال شد', 'watchdog');
    }
  };

  Sim.prototype.evModule = function (node) {
    var self = this, best = null;
    // gather requests from vehicles on inbound links of this node
    for (var s = 0; s < 4; s++) {
      var Ls2 = node.inL[s]; if (!Ls2) continue;
      Ls2.forEach(function (L) { L.vehicles.forEach(function (v) {
        if (!v.isEV) return;
        var d = L.len - v.s;
        if (!v.detected) {
          if (v.spec.gps && d <= GPS_RANGE) {
            v.detected = 'gps';
            self.evSay('اولویت اورژانس: ' + KIND_FA[v.kind] + ' از ورودی ' + SIDE_FA[s] + ' با GPS امضاشده شناسایی شد (' + Math.round(d) + ' متر تا چراغ)', 'ev');
          } else if (d <= VIS_RANGE) {
            if (self.rng() < (self.opts.fault ? 0.1 : 0.7)) v.visT += 1;
            if (self.rng() < 0.8) v.audT += 1;
            if (v.visT >= 1 && v.audT >= 1) {
              v.detected = 'va';
              self.evSay('اولویت اورژانس: ' + KIND_FA[v.kind] + ' از ورودی ' + SIDE_FA[s] + ' با دید + صدا تأیید شد (گیت AND، ' + Math.round(d) + ' متر)', 'ev');
            }
          }
        }
        if (v.detected) {
          var cand = { v: v, side: s, rank: EVRANK[v.kind], d: d };
          if (!best || cand.rank > best.rank || (cand.rank === best.rank && cand.d < best.d)) best = cand;
        }
      }); });
    }
    var sg = node.sig;
    if (best) {
      if (!node.pre || node.pre.v !== best.v) {
        node.pre = { v: best.v, side: best.side, since: this.t, turn: best.v.nextConn ? best.v.nextConn.turn : 0 };
        this.stats.preempts++;
        this.evSay('هسته‌ی ایمنی: پیش‌دستی چراغ چهارراه ' + (node.id + 1) + ' برای ' + KIND_FA[best.v.kind] + ' (ورودی ' + SIDE_FA[best.side] + ')، دنباله‌ی امن: زرد ← تمام‌قرمز ← سبز', 'kernel');
      }
      node.preRel = 0;
    } else if (node.pre) {
      node.preRel += 1;
      if (node.preRel >= 3) {
        node.pre = null; sg.hold = false;
        this.evSay('پیش‌دستی چهارراه ' + (node.id + 1) + ' تمام شد ← بازگشت به کنترل عادی', 'info');
      }
    }
    if (node.pre) {
      var p = node.phaseFor(node.pre.side, node.pre.turn);
      if (sg.state === 'G' && sg.phase === p) {
        sg.hold = true; sg.holdT += 1;
        if (sg.holdT > EV_HOLD_MAX) { sg.hold = false; node.pre = null; }
      } else if (sg.state === 'G') {
        this.request(node, p, 'emergency', true, false);
      } else if (sg.target !== p) {
        sg.target = p;
      }
      return true;
    }
    return false;
  };

  Sim.prototype.smartCtl = function (node) {
    this.perceive(node); this.watchdog(node);
    var sg = node.sig;
    if (this.opts.evModule && this.evModule(node)) { if (sg.hold) return; return; }
    else sg.hold = false;
    if (sg.state !== 'G') return;
    if (node.fallback) { this.fixedCtl(node, 30, 10); return; }
    var np = node.phases.length, cur = sg.phase, P = [], Wt = [], p;
    for (p = 0; p < np; p++) { P[p] = this.phasePressure(node, p); Wt[p] = this.phaseWait(node, p); }
    var Pc = P[cur], best = -1, bp = -1e9, wi = -1, wv = 0;
    for (p = 0; p < np; p++) { if (p === cur) continue; if (P[p] > bp) { bp = P[p]; best = p; } if (Wt[p] > wv) { wv = Wt[p]; wi = p; } }
    if (best < 0) return;
    node.lastPressure = [Pc, bp];
    var cph = node.phases[cur], maxG = cph.kind === 'L' ? TUNE.maxL : TUNE.maxT, why = '', target = best;
    if (this.phaseNear(node, cur) === 0) sg.gapT = (sg.gapT || 0) + 1; else sg.gapT = 0;
    if (sg.gapT >= TUNE.gap && bp > (node.phases[best].kind === 'L' ? TUNE.leftThr : 0.8)) why = 'صف این فاز خالی شد (gap-out)';
    else if (bp > TUNE.ratio * Pc + TUNE.off && (node.phases[best].kind !== 'L' || bp > TUNE.leftThr)) why = 'فشار ترافیک فاز بعدی بیشتر است';
    else if (sg.greenElapsed >= maxG && bp > 0.5) why = 'سقف سبز رسید';
    else if (wv >= MAX_WAIT && sg.greenElapsed >= (cph.kind === 'L' ? 12 : 30)) { why = 'سقف انتظار یک ورودی رسید (ضد گرسنگی)'; target = wi; }
    if (why) {
      var ok = this.request(node, target, 'decider', false, false);
      if (ok) this.say('چهارراه ' + (node.id + 1) + ': رفتن به «' + node.phases[target].name + '» — ' + why + ' (فشار ' + Pc.toFixed(1) + ' ← ' + P[target].toFixed(1) + ')', 'decide', null);
    } else if (this.t - (this.lastLogAt['hold' + node.id] || -99) > 15 && Pc > 3) {
      this.lastLogAt['hold' + node.id] = this.t;
      this.say('چهارراه ' + (node.id + 1) + ': «' + cph.name + '» ادامه می‌یابد — فشار ' + Pc.toFixed(1) + ' در برابر ' + bp.toFixed(1), 'decide');
    }
  };

  /* --- vehicle physics --- */
  Sim.prototype.sameLinkLeader = function (veh) {
    var L = veh.link, best = null, bestGap = Infinity, crawl = false;
    for (var i = 0; i < L.vehicles.length; i++) {
      var o = L.vehicles[i]; if (o === veh) continue;
      var ahead = o.s > veh.s || (o.s === veh.s && o.id < veh.id);
      if (!ahead) continue;
      var g = (o.s - o.len) - veh.s;
      if (veh.isEV && !o.isEV) { if (g < 12 && o.v < 4) crawl = true; continue; }
      if (g < bestGap) { bestGap = g; best = o; }
    }
    return { lead: best, gap: bestGap, crawl: crawl };
  };

  Sim.prototype.lookAhead = function (veh) {
    var L = veh.link, c = veh.nextConn, gap = Infinity, v = 0;
    var base = L.len - veh.s;
    var consider = function (arr, off) {
      for (var i = 0; i < arr.length; i++) {
        var o = arr[i]; if (veh.isEV && !o.isEV) continue;
        var g = off + (o.s - o.len);
        if (g < gap) { gap = g; v = o.v; }
      }
    };
    if (c) { consider(c.vehicles, base); consider(c.outLink.vehicles, base + c.len); }
    return { gap: gap, v: v };
  };

  Sim.prototype.exitFree = function (c, veh) {
    var arr = c.outLink.vehicles, need = veh.len + 2.5, i;
    for (i = 0; i < arr.length; i++) if ((arr[i].s - arr[i].len) < need) return false;
    for (i = 0; i < c.vehicles.length; i++) if ((c.vehicles[i].s - c.vehicles[i].len) < veh.len + 1.0) return false;
    return true;
  };
  Sim.prototype.connFree = function (c, veh, strict) {
    var node = c.node, i, j;
    for (i = 0; i < c.conflicts.length; i++) if (c.conflicts[i].vehicles.length) return false;
    if (c.turn === 3) {
      var Los = node.inL[opp(c.a)];
      if (Los && this.light(node, opp(c.a), 0) !== 'R') {
        for (var lq = 0; lq < Los.length; lq++) { var Lo = Los[lq];
          for (j = 0; j < Lo.vehicles.length; j++) {
            var o = Lo.vehicles[j];
            if (o.nextConn && o.nextConn.turn === 0) {
              var d = Lo.len - o.s, tt = d / Math.max(o.v, 2);
              if (d < 50 && tt < 3.2) return false;
            }
          }
        }
      }
    }
    if (strict) {
      for (i = 0; i < c.conflicts.length; i++) {
        var Li = c.conflicts[i].inLink; if (!Li) continue;
        for (j = 0; j < Li.vehicles.length; j++) {
          var w = Li.vehicles[j];
          if (Li.len - w.s < 35 && w.v > 2) return false;
        }
      }
    }
    return true;
  };

  Sim.prototype.canCross = function (veh) {
    var L = veh.link, node = L.toNode, a = L.toSide, c = veh.nextConn;
    var sig = this.light(node, a, c.turn), dist = L.len - veh.s, ok = false;
    if (sig === 'G') ok = true;
    else if (sig === 'Y') { var stopD = veh.v * veh.v / (2 * 4.5) + veh.v * 0.3; ok = dist < stopD && veh.v > 1; }
    var redRun = false;
    if (!ok && veh.isEV && this.mode === 'fixed' && this.opts.riskRed && veh.redWait >= 5 &&
        this.connFree(c, veh, true) && this.exitFree(c, veh)) { ok = true; redRun = true; }
    if (!ok) return false;
    var chk = dist < Math.max(10, veh.v * veh.v / 8 + 6);
    if (chk && !(this.connFree(c, veh, false) && this.exitFree(c, veh))) return false;
    if (redRun && dist < 3) { veh.risky = true; }
    return true;
  };

  Sim.prototype.pickConn = function (veh, link) {
    var node = link.toNode, a = link.toSide, ln = link.lane, want = veh.spec.turns[Math.min(veh.hop, 7)];
    var order = [want, 0, 1, 3], i;
    for (i = 0; i < order.length; i++) {
      var h = opp(a), b = (h + order[i]) % 4;
      if (node.sides[b] && b !== a && node.conn[a][ln][b]) return node.conn[a][ln][b];
    }
    for (var b2 = 0; b2 < 4; b2++) if (node.conn[a][ln][b2]) return node.conn[a][ln][b2];
    return null;
  };

  Sim.prototype.chooseLane = function (node, a, want, loadFn) {
    var order = [want, 0, 1, 3], h = opp(a), nl = node.inL[a].length;
    for (var i = 0; i < order.length; i++) {
      var b = (h + order[i]) % 4; if (!node.sides[b] || b === a) continue;
      var best = -1, bl = 1e9;
      for (var ln = 0; ln < nl; ln++) { if (!node.conn[a][ln][b]) continue; var l = loadFn(ln); if (l < bl) { bl = l; best = ln; } }
      if (best >= 0) return { lane: best, conn: node.conn[a][best][b] };
    }
    return null;
  };
  Sim.prototype.place = function (veh, link, s) {
    if (veh.link) { var ix = veh.link.vehicles.indexOf(veh); if (ix >= 0) veh.link.vehicles.splice(ix, 1); }
    var conn = null;
    if (link.kind === 'internal') {
      var node = link.toNode, a = link.toSide, want = veh.spec.turns[Math.min(veh.hop, 7)], cur = link.lane;
      var ch = this.chooseLane(node, a, want, function (ln) {
        var Lk = node.inL[a][ln], c = Lk.vehicles.length;
        for (var k = 0; k < Lk.vehicles.length; k++) if (Lk.vehicles[k].s - Lk.vehicles[k].len < s + veh.len + 1.5) { c += 6; break; }
        return c + (ln === cur ? -0.4 : 0);
      });
      if (ch) { var tl = node.inL[a][ch.lane]; if (tl !== link) { veh.latOff = (link.lane - tl.lane) * LW; link = tl; } conn = ch.conn; }
    }
    veh.link = link; veh.s = s; link.vehicles.push(veh);
    if (link.kind === 'entry') veh.nextConn = this.pickConn(veh, link);
    else if (link.kind === 'internal') veh.nextConn = conn || this.pickConn(veh, link);
    else veh.nextConn = null;
  };

  Sim.prototype.moveVeh = function (veh, dt) {
    var L = veh.link, isConn = L.kind === 'conn', node = L.toNode;
    var vdes = L.vlim * veh.vf; if (veh.vcap) vdes = Math.min(vdes, veh.vcap);
    var gap = Infinity, vl = 0, leadObj = null;
    var sl = this.sameLinkLeader(veh);
    if (sl.lead) { gap = sl.gap; vl = sl.lead.v; leadObj = sl.lead; }
    if (sl.crawl) vdes = Math.min(vdes, CRAWL);
    var sig = null, permit = true;
    if (!isConn && node) {
      sig = this.light(node, L.toSide, veh.nextConn ? veh.nextConn.turn : 0);
      if (veh.isEV && veh.v < 0.6 && L.len - veh.s < 3.5 && sig !== 'G') veh.redWait += dt;
      else if (sig === 'G') veh.redWait = 0;
      permit = veh.nextConn ? this.canCross(veh) : false;
      if (!permit) { var g2 = (L.len - veh.s) - 0.6; if (g2 < gap) { gap = g2; vl = 0; leadObj = null; } }
      else { var la = this.lookAhead(veh); if (la.gap < gap) { gap = la.gap; vl = la.v; leadObj = null; } }
      if (veh.risky) vdes = Math.min(vdes, 4);
    } else if (isConn) {
      var oL = L.outLink, base = L.len - veh.s;
      for (var i = 0; i < oL.vehicles.length; i++) {
        var o = oL.vehicles[i]; if (veh.isEV && !o.isEV) continue;
        var g = base + (o.s - o.len); if (g < gap) { gap = g; vl = o.v; leadObj = null; }
      }
    }
    var vs = Infinity;
    if (gap < Infinity) { var gg = Math.max(0, gap - MINGAP); vs = -B * TAU + Math.sqrt(B * B * TAU * TAU + vl * vl + 2 * B * gg); }
    var vnew = Math.min(veh.v + veh.acc * dt, vdes, vs);
    vnew = Math.max(vnew, Math.max(0, veh.v - 9 * dt));
    var ds = (veh.v + vnew) / 2 * dt, ns = veh.s + ds;
    if (leadObj) { var lim = (leadObj.s - leadObj.len) - 0.3; if (ns > lim) { ns = Math.max(veh.s, lim); } }
    veh.v = vnew;
    if (veh.latOff) { var dl = 3 * dt; veh.latOff = Math.abs(veh.latOff) <= dl ? 0 : veh.latOff - Math.sign(veh.latOff) * dl; }
    // stopped-time accounting
    var stoppedNow = vnew < 0.5 && !isConn;
    if (stoppedNow) { veh.stopped += dt; veh.curWait += dt; if (!veh.wasStopped) { veh.stops++; veh.wasStopped = true; } }
    else { if (vnew > 2) { veh.curWait = 0; veh.wasStopped = false; } }
    // transitions
    if (L.kind === 'exit') {
      veh.s = ns;
      if (veh.s >= L.len) this.finish(veh);
      return;
    }
    if (isConn) {
      veh.s = ns;
      if (veh.s >= L.len) {
        var over = veh.s - L.len; veh.hop++; this.place(veh, L.outLink, over);
      }
      return;
    }
    // inbound link
    if (ns >= L.len) {
      if (permit && veh.nextConn && this.canCrossNow(veh)) {
        var c = veh.nextConn;
        if (this.light(node, L.toSide, c.turn) === 'R') this.stats.redRuns += veh.risky ? 0 : 1;
        if (veh.risky && !veh.riskCounted) { veh.riskCounted = true; this.stats.risky++; this.evSay('خطر: ' + KIND_FA[veh.kind] + ' در وضع فعلی چراغ قرمز را با احتیاط رد کرد', 'risk'); }
        var over2 = ns - L.len; this.place(veh, c, over2);
      } else { veh.s = L.len - 0.01; veh.v = 0; }
    } else veh.s = ns;
  };
  Sim.prototype.canCrossNow = function (veh) {
    var c = veh.nextConn; var L = veh.link, sig = this.light(L.toNode, L.toSide, c.turn);
    if (sig === 'R' && !veh.risky) { if (!(veh.isEV && this.mode === 'fixed' && this.opts.riskRed && veh.redWait >= 5)) return false; }
    return this.connFree(c, veh, veh.risky) && this.exitFree(c, veh);
  };

  Sim.prototype.finish = function (veh) {
    var ix = veh.link.vehicles.indexOf(veh); if (ix >= 0) veh.link.vehicles.splice(ix, 1);
    var jx = this.vehs.indexOf(veh); if (jx >= 0) this.vehs.splice(jx, 1);
    var tt = this.t - veh.t0;
    this.stats.n++; this.stats.sumStop += veh.stopped; this.stats.sumTravel += tt; this.stats.sumStops += veh.stops;
    this.stats.recent.push({ t: this.t, stop: veh.stopped });
    if (veh.isEV) this.stats.ev.push({ kind: veh.kind, time: tt, stopped: veh.stopped, risky: veh.risky, id: veh.id });
  };

  Sim.prototype.inject = function (spec) {
    var v = new Veh(spec, this.t), A = this.net.entries[spec.entry], pq = this.pending[spec.entry], t0 = spec.turns[0];
    var ch = this.chooseLane(A.toNode, A.toSide, t0, function (l) { return A.lanes[l].vehicles.length + pq[l].length + l * 0.01; });
    var lane = ch ? ch.lane : 0;
    v.lane = lane; pq[lane].push(v);
  };

  Sim.prototype.step = function (dt, specs) {
    var self = this;
    this.t += dt;
    for (var i = 0; i < specs.length; i++) this.inject(specs[i]);
    // spawn
    this.net.entries.forEach(function (A, ei) { for (var ln = 0; ln < A.lanes.length; ln++) {
      var E = A.lanes[ln], q = self.pending[ei][ln];
      q.forEach(function (v) { v.stopped += dt; });
      while (q.length) {
        var v = q[0], free = true;
        for (var k = 0; k < E.vehicles.length; k++) if ((E.vehicles[k].s - E.vehicles[k].len) < v.len + MINGAP + 2.5) { free = false; break; }
        if (!free) break;
        q.shift(); v.v = Math.min(E.vlim * v.vf, 10); v.spawned = true;
        self.vehs.push(v); self.place(v, E, 0);
      }
    } });
    // controllers (1 Hz)
    this.acc1 += dt;
    if (this.acc1 >= 1) {
      this.acc1 -= 1;
      this.net.nodes.forEach(function (n) {
        if (self.mode === 'fixed') self.fixedCtl(n, self.opts.greenFixed, self.opts.leftFixed);
        else self.smartCtl(n);
      });
    }
    this.net.nodes.forEach(function (n) { self.advanceSignal(n, dt); });
    // move vehicles: per link, front to back
    var all = this.net.links.concat(this.net.conns);
    all.forEach(function (L) {
      if (!L.vehicles.length) return;
      var arr = L.vehicles.slice().sort(function (a, b) { return b.s - a.s || a.id - b.id; });
      for (var i = 0; i < arr.length; i++) if (arr[i].link === L) self.moveVeh(arr[i], dt);
    });
    // trim recent window
    var rc = this.stats.recent; while (rc.length && rc[0].t < this.t - 300) rc.shift();
  };

  /* ---------- metrics & geometry helpers for the UI ---------- */
  Sim.prototype.metrics = function () {
    var q = 0, vs = 0, nv = 0, maxW = 0, i;
    for (i = 0; i < this.vehs.length; i++) {
      var v = this.vehs[i];
      if (v.link.kind !== 'conn') { nv++; vs += v.v; if (v.v < 0.5) q++; }
      if (v.curWait > maxW) maxW = v.curWait;
    }
    var pend = 0; this.pending.forEach(function (p) { p.forEach(function (q) { pend += q.length; }); });
    var rc = this.stats.recent, sum = 0, cnt = 0, thr = 0;
    for (i = 0; i < rc.length; i++) { if (rc[i].t >= this.t - 120) { sum += rc[i].stop; cnt++; } }
    thr = rc.length;
    var evs = this.stats.ev, evAvg = null;
    if (evs.length) { evAvg = evs.reduce(function (a, e) { return a + e.time; }, 0) / evs.length; }
    // stricter delay: also counts the waiting time of vehicles that have not finished yet (no survivor bias in overload)
    var allSum = this.stats.sumStop, allN = this.stats.n + pend;
    for (i = 0; i < this.vehs.length; i++) { allSum += this.vehs[i].stopped; allN++; }
    this.pending.forEach(function (p) { p.forEach(function (q) { q.forEach(function (v) { allSum += v.stopped; }); }); });
    return {
      avgStopAll: allN ? allSum / allN : 0,
      avgStop: this.stats.n ? this.stats.sumStop / this.stats.n : 0,
      recentStop: cnt ? sum / cnt : 0,
      queue: q + pend, inNet: this.vehs.length, pending: pend,
      thr5: thr, maxWait: maxW, avgKmh: nv ? (vs / nv) * 3.6 : 0,
      done: this.stats.n, evAvg: evAvg, evN: evs.length, risky: this.stats.risky,
      rejected: this.stats.rejected, preempts: this.stats.preempts, stops: this.stats.n ? this.stats.sumStops / this.stats.n : 0
    };
  };

  Sim.prototype.pose = function (veh) {
    var L = veh.link, x, y, ang;
    if (L.kind === 'conn') {
      var s = Math.min(Math.max(veh.s, 0), L.len), i = 1;
      while (i < L.cum.length - 1 && L.cum[i] < s) i++;
      var s0 = L.cum[i - 1], s1 = L.cum[i], f = s1 > s0 ? (s - s0) / (s1 - s0) : 0;
      var p0 = L.pts[i - 1], p1 = L.pts[i];
      x = p0.x + (p1.x - p0.x) * f; y = p0.y + (p1.y - p0.y) * f; ang = Math.atan2(p1.y - p0.y, p1.x - p0.x);
    } else {
      x = L.ax + L.dx * veh.s; y = L.ay + L.dy * veh.s; ang = Math.atan2(L.dy, L.dx);
    }
    if (veh.latOff && L.kind !== 'conn') { x += -Math.sin(ang) * -veh.latOff; y += Math.cos(ang) * -veh.latOff; }
    if (veh.isEV) { x += Math.sin(ang) * 1.3; y -= Math.cos(ang) * 1.3; }
    return { x: x, y: y, ang: ang };
  };

  /* ---------- world: two sims, one demand ---------- */
  function World(key, opts, seed) {
    this.key = key; this.seed = seed || 1;
    this.opts = Object.assign({ leftMode: 'protected', leftFixed: 12, greenFixed: 35, riskRed: true, evModule: true, coopt: true, fault: false }, opts || {});
    this.base = new Sim(key, 'fixed', this.opts);
    this.smart = new Sim(key, 'smart', this.opts);
    this.demand = new Demand(this.base.net, this.seed);
    this.t = 0; this.extra = [];
    this.hist = [];
    this.lastHist = 0;
  }
  World.prototype.step = function (dt) {
    this.t += dt;
    var specs = this.demand.step(this.t, dt);
    if (this.extra.length) { specs = specs.concat(this.extra); this.extra = []; }
    this.base.step(dt, specs); this.smart.step(dt, specs);
    if (this.t - this.lastHist >= 5) {
      this.lastHist = this.t;
      var a = this.base.metrics(), b = this.smart.metrics();
      this.hist.push({ t: this.t, a: a.recentStop, b: b.recentStop, qa: a.queue, qb: b.queue });
      if (this.hist.length > 240) this.hist.shift();
    }
  };
  World.prototype.resetStats = function () { this.base.resetStats(); this.smart.resetStats(); this.hist = []; this.lastHist = this.t; };
  World.prototype.addEV = function (kind) {
    var spec = this.demand.makeEV(kind, null);
    this.extra.push(spec);
    var E = this.base.net.entries[spec.entry];
    return { spec: spec, label: E.label };
  };
  World.prototype.setRate = function (r) { this.demand.setRate(r, this.t); };
  World.prototype.surge = function () {
    var idx = this.demand.triggerSurge(this.t);
    return this.base.net.entries[idx].label;
  };

  return {
    TUNE: TUNE, SCENARIOS: SCENARIOS, World: World, Sim: Sim, Demand: Demand, buildNetwork: buildNetwork,
    KIND_FA: KIND_FA, SIDE_FA: SIDE_FA, EVRANK: EVRANK,
    consts: { R: R, LW: LW, SPACING: SPACING, ENTRY: ENTRY, YELLOW: YELLOW, ALLRED: ALLRED, MIN_GREEN: MIN_GREEN, MAX_GREEN: MAX_GREEN }
  };
});
