/* EVP-12 intersection simulation (metres, x = east, y = north, z = up).
 * Shows how the vehicle transmits (IR 12 Hz frames + signed LoRa/GPS beacon), how the intersection
 * receives (directional detector heads + LoRa antenna) and that only the approach the vehicle is
 * really on gets a green. The decision rules mirror firmware/common/evp_logic.cpp. */
window.EVPSim = function (THREE, scene) {
  const D2R = Math.PI / 180;
  const G = new THREE.Group(); G.visible = false; scene.add(G);

  // ---------------------------------------------------------------- geometry of the junction
  const NAMES = ['شمال', 'شرق', 'جنوب', 'غرب'];
  const TRAVEL = [[0, -1], [-1, 0], [0, 1], [1, 0]];          // direction vehicles drive on each approach
  const APP = TRAVEL.map((d, i) => {
    const u = [-d[0], -d[1]], r = [d[1], -d[0]];               // upstream, right-hand side
    const stop = [u[0] * 12 + r[0] * 3.5, u[1] * 12 + r[1] * 3.5];
    const head = [u[0] * 13 + r[0] * 8.5, u[1] * 13 + r[1] * 8.5];
    const bearing = (Math.atan2(d[0], d[1]) / D2R + 360) % 360;
    return { i, name: NAMES[i], d, u, r, stop, head, bearing, phase: i % 2 };
  });
  const at = (a, s) => [a.stop[0] + a.u[0] * s, a.stop[1] + a.u[1] * s];
  const CAB = [15, -15];
  const FOV = 12, EMIT = 15, PERIOD = 1 / 12;
  const LEVELS = ['پایین', 'متوسط', 'بالا'], PATTERN = [2, 1, 2, 0, 2, 1];

  const SCEN = [
    { id: 'N', fa: 'آمبولانس از شمال', from: 0, expect: 0,
      desc: 'آمبولانس با آژیر روشن از ۳۸۰ متری شمال به سمت تقاطع می‌آید.',
      exp: 'فقط چراغ مسیر شمال سبز می‌شود. سه مسیر دیگر قرمز می‌مانند.' },
    { id: 'E', fa: 'آمبولانس از شرق', from: 1, expect: 1,
      desc: 'همان آمبولانس، این بار از خیابان شرقی.',
      exp: 'فقط چراغ مسیر شرق سبز می‌شود.' },
    { id: 'PAR', fa: 'خیابان موازی', expect: -1, start: [76.5, 392], dir: [0, -1], dist: 600,
      desc: 'آمبولانس در خیابانی موازی، ۸۰ متر شرق‌تر، به سمت جنوب می‌رود. هم‌جهت با مسیر شمال است، اما در خیابان دیگری است.',
      exp: 'هیچ چراغی عوض نمی‌شود. خودرو بیرون از دید هدها و بیرون از کریدور GPS است.' },
    { id: 'OUT', fa: 'دور شدن از تقاطع', expect: -1, start: [3.5, 14], dir: [0, 1], dist: 400,
      desc: 'آمبولانس از تقاطع خارج شده و در همان خیابان شمالی دور می‌شود.',
      exp: 'درخواستی صادر نمی‌شود. فرستنده رو به جلو است، پس هیچ هدی نور را نمی‌گیرد. جهت حرکت GPS هم خلاف ورود به تقاطع است.' },
    { id: 'REF', fa: 'بازتاب از ساختمان', from: 3, expect: 3, reflect: 0,
      desc: 'آمبولانس از غرب می‌آید. نما‌ی شیشه‌ای ساختمان گوشه بخشی از نور را به هد شمال بازتاب می‌دهد.',
      exp: 'فقط چراغ غرب سبز می‌شود. بازتاب ضعیف روی هد شمال رد می‌شود.' },
    { id: 'GPS', fa: 'قطع GPS', from: 2, expect: 2, gps: false,
      desc: 'آمبولانس از جنوب می‌آید، اما GPS در دره‌ی شهری بین ساختمان‌های بلند فیکس ندارد.',
      exp: 'فقط چراغ جنوب سبز می‌شود. کانال IR به‌تنهایی، بعد از یک ثانیه دریافت پیوسته، درخواست را تأیید می‌کند.' },
    { id: 'OFF', fa: 'بدون آژیر', from: 0, expect: -1, siren: false,
      desc: 'همان خودرو از شمال می‌آید، ولی آژیر و چراغ گردان خاموش است.',
      exp: 'فرستنده هیچ چیزی ارسال نمی‌کند و چرخه‌ی عادی چراغ‌ها ادامه پیدا می‌کند.' },
  ];

  // ---------------------------------------------------------------- scene
  const M = (color, extra) => new THREE.MeshStandardMaterial(Object.assign({ color, roughness: 0.85, metalness: 0.0 }, extra || {}));
  const boxAt = (w, d, h, x, y, z, mat) => { const m = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), mat); m.position.set(x, y, z); G.add(m); return m; };
  boxAt(1400, 1400, 0.2, 0, 0, -0.15, M(0x9aa69a));
  const asphalt = M(0x3b4045), paint = M(0xe9ecef), dashMat = M(0xd8c25a);
  boxAt(14, 1100, 0.1, 0, 0, 0, asphalt); boxAt(1100, 14, 0.1, 0, 0, 0.01, asphalt);
  boxAt(8, 1100, 0.1, 80, 0, 0, asphalt);
  for (let k = -540; k < 540; k += 12) {
    if (Math.abs(k) > 16) { boxAt(0.25, 6, 0.04, 0, k, 0.07, dashMat); boxAt(6, 0.25, 0.04, k, 0, 0.07, dashMat); }
    boxAt(0.2, 6, 0.04, 80, k, 0.07, paint);
  }
  const bmat = M(0xb9bfc6), glass = M(0x6f93b5, { roughness: 0.2, metalness: 0.3 });
  [[-40, 40, 26, glass], [40, 40, 14, bmat], [40, -45, 18, bmat], [-42, -42, 22, bmat], [120, 120, 30, bmat], [120, -100, 16, bmat], [-120, 140, 20, bmat]]
    .forEach(([x, y, h, m]) => boxAt(36, 36, h, x, y, h / 2, m));

  // stop-line bars, poles, signal heads, detector heads, FOV wedges, GPS corridors
  const lampColor = { G: 0x22c55e, Y: 0xf5b301, R: 0xe5322d };
  const stopBars = [], lamps = [], heads = [], wedges = [], corridors = [];
  APP.forEach(a => {
    const bar = new THREE.Mesh(new THREE.BoxGeometry(Math.abs(a.u[0]) ? 0.8 : 7, Math.abs(a.u[0]) ? 7 : 0.8, 0.08), new THREE.MeshBasicMaterial({ color: lampColor.R }));
    bar.position.set(a.stop[0] - a.u[0] * 0.5, a.stop[1] - a.u[1] * 0.5, 0.1); G.add(bar); stopBars.push(bar);
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.18, 6, 12), M(0x8a9097, { metalness: 0.4 }));
    pole.rotation.x = Math.PI / 2; pole.position.set(a.head[0], a.head[1], 3); G.add(pole);
    const yaw = Math.atan2(a.u[1], a.u[0]);
    const sig = new THREE.Group(); sig.position.set(a.head[0], a.head[1], 4.2); sig.rotation.z = yaw; G.add(sig);
    const housing = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.45, 1.3), M(0x1d2125)); sig.add(housing);
    const ls = {};
    ['R', 'Y', 'G'].forEach((c, j) => {
      const l = new THREE.Mesh(new THREE.SphereGeometry(0.16, 16, 10), new THREE.MeshBasicMaterial({ color: 0x2a2e33 }));
      l.position.set(0.22, 0, 0.42 - j * 0.42); sig.add(l); ls[c] = l;
    });
    lamps.push(ls);
    const hg = new THREE.Group(); hg.position.set(a.head[0], a.head[1], 5.6); hg.rotation.z = yaw; G.add(hg);
    const hb = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.32, 0.26), new THREE.MeshStandardMaterial({ color: 0x16181b, emissive: 0x000000 })); hb.position.x = 0.25; hg.add(hb);
    heads.push(hb);
    // field of view wedge (±12°, 210 m) on the road surface
    const sh = new THREE.Shape(); const L = 210;
    sh.moveTo(0, 0); sh.lineTo(L * Math.cos(FOV * D2R), L * Math.sin(FOV * D2R)); sh.lineTo(L * Math.cos(FOV * D2R), -L * Math.sin(FOV * D2R)); sh.lineTo(0, 0);
    const w = new THREE.Mesh(new THREE.ShapeGeometry(sh), new THREE.MeshBasicMaterial({ color: 0xf0ab1f, transparent: true, opacity: 0.10, depthWrite: false }));
    w.position.set(a.head[0], a.head[1], 0.2); w.rotation.z = yaw; G.add(w); wedges.push(w);
    // GPS corridor (signed GPS must fall inside it, heading within ±30°)
    const c = new THREE.Mesh(new THREE.PlaneGeometry(600, 50), new THREE.MeshBasicMaterial({ color: 0x3b82f6, transparent: true, opacity: 0.07, depthWrite: false }));
    const mid = at(a, 300); const cen = [mid[0] - a.r[0] * 3.5, mid[1] - a.r[1] * 3.5];
    c.position.set(cen[0], cen[1], 0.15); c.rotation.z = yaw; G.add(c); corridors.push(c);
  });
  // cabinet + LoRa antenna
  boxAt(1.0, 0.6, 1.5, CAB[0], CAB[1], 0.75, M(0x7a828a, { metalness: 0.3 }));
  const ant = boxAt(0.08, 0.08, 2.2, CAB[0], CAB[1], 2.6, new THREE.MeshStandardMaterial({ color: 0x222222, emissive: 0x000000 }));

  // vehicle
  const car = new THREE.Group(); G.add(car);
  const body = new THREE.Mesh(new THREE.BoxGeometry(5.4, 2.1, 2.1), M(0xf4f6f8)); body.position.z = 1.25; car.add(body);
  const stripe = new THREE.Mesh(new THREE.BoxGeometry(5.42, 2.12, 0.3), M(0xd32f2f)); stripe.position.z = 1.1; car.add(stripe);
  const barR = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.8, 0.22), new THREE.MeshBasicMaterial({ color: 0xff2020 })); barR.position.set(1.6, 0.45, 2.42); car.add(barR);
  const barB = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.8, 0.22), new THREE.MeshBasicMaterial({ color: 0x2060ff })); barB.position.set(1.6, -0.45, 2.42); car.add(barB);
  const vtu = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.45, 0.3), M(0x202326)); vtu.position.set(2.2, 0, 2.47); car.add(vtu);
  const halo = new THREE.Mesh(new THREE.RingGeometry(7, 8.5, 48), new THREE.MeshBasicMaterial({ color: 0xf0ab1f, transparent: true, opacity: 0.55, depthWrite: false })); halo.position.z = 0.25; car.add(halo);
  const beamGeo = new THREE.ConeGeometry(Math.tan(EMIT * D2R) * 70, 70, 24, 1, true); beamGeo.translate(0, -35, 0); beamGeo.rotateZ(Math.PI / 2);
  const beam = new THREE.Mesh(beamGeo, new THREE.MeshBasicMaterial({ color: 0xff3030, transparent: true, opacity: 0.0, depthWrite: false, side: THREE.DoubleSide }));
  beam.position.set(2.5, 0, 2.47); car.add(beam);

  // transient effects
  const lineMat = (c, o) => new THREE.LineBasicMaterial({ color: c, transparent: true, opacity: o });
  const mkLine = (c) => { const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]); const l = new THREE.Line(g, lineMat(c, 0)); G.add(l); return l; };
  const rxLines = APP.map(() => mkLine(0xf0ab1f));
  const refLine = (() => { const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]); const l = new THREE.Line(g, lineMat(0xa855f7, 0)); G.add(l); return l; })();
  const loraLine = mkLine(0x3b82f6);
  const rings = Array.from({ length: 4 }, () => { const m = new THREE.Mesh(new THREE.RingGeometry(0.96, 1, 96), new THREE.MeshBasicMaterial({ color: 0x3b82f6, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide })); m.position.z = 1; G.add(m); return { m, age: 99 }; });

  // ---------------------------------------------------------------- decision engine (port of evp_logic.cpp)
  const CFG = { gpsEta: 25, preEta: 45, hold: 6, maxReq: 120, lockout: 20, passed: -12, halfW: 25, maxD: 600, hdgTol: 30, need: 0.4, needIr: 1.0 };
  function irStatus(E, h, now, need) {
    const L = E.frames[h].filter(f => now - f.t <= 1.0);
    const st = { n: L.length, periodic: 0, score: 0, best: -1, run: 0, ok: false };
    let runStart = 0, last = -1;
    E.frames[h].forEach(f => { if (now - f.t > 2.5) return; if (last < 0 || f.t - last > 0.4) runStart = f.t; last = f.t; });
    L.forEach((f, i) => {
      st.score += f.lvl === 0 ? 9 : f.lvl === 1 ? 3 : 1;
      if (st.best < 0 || f.lvl < st.best) st.best = f.lvl;
      if (i) { const dt = f.t - L[i - 1].t, m = Math.round(dt / PERIOD); if (m >= 1 && m <= 4 && Math.abs(dt - m * PERIOD) <= 0.004) st.periodic++; }
    });
    if (last >= 0 && now - last <= 0.3) st.run = last - runStart;
    st.ok = st.run >= need && st.periodic >= 3;
    return st;
  }
  function bestIr(E, now, need) {
    let best = -1, bs = 0, second = 0;
    for (let h = 0; h < 4; h++) { const s = irStatus(E, h, now, need); if (!s.ok) continue; if (s.score > bs) { second = bs; bs = s.score; best = h; } else if (s.score > second) second = s.score; }
    return { ch: best, amb: best >= 0 && second >= 0.6 * bs };
  }
  function locate(T, p, hdg, spd, gpsGood) {
    T.gpsGood = gpsGood; T.speed = spd; T.heading = hdg;
    let found = -1, fs = 0;
    if (gpsGood) APP.forEach(a => {
      const e = p[0] - a.stop[0], n = p[1] - a.stop[1];
      const s = e * a.u[0] + n * a.u[1], l = Math.abs(e * a.u[1] - n * a.u[0]);
      let dh = ((hdg - a.bearing) % 360 + 540) % 360 - 180;
      const ok = spd < 2 || Math.abs(dh) <= CFG.hdgTol;
      if (s >= CFG.passed && s <= CFG.maxD && l <= CFG.halfW && ok && (found < 0 || s < fs)) { found = a.i; fs = s; }
    });
    if (found >= 0) {
      T.nMatch = T.app === found ? T.nMatch + 1 : 1; T.prevS = T.app === found ? T.s : 1e9;
      T.app = found; T.s = fs; T.eta = fs > 0 ? fs / Math.max(spd, 2) : 0;
    } else {
      if (T.app >= 0 && T.s < 40) T.s = CFG.passed - 1;
      T.app = -1; T.nMatch = 0; T.eta = -1;
    }
  }
  function engineTick(E, now, log) {
    const T = E.track;
    const fresh = T && now - T.t <= 3;
    const loraApp = fresh && T.gpsGood ? T.app : -1;
    const ir = bestIr(E, now, CFG.need);
    let cand = null; E.why = '';
    if (loraApp >= 0 && ir.ch >= 0) {
      if (loraApp === ir.ch) cand = { app: loraApp, src: 'IR+GPS' };
      else conflict(E, now, log, ir.ch, loraApp);
    } else if (loraApp >= 0 && T.nMatch >= 2 && T.s < T.prevS + 1 && T.eta <= CFG.gpsEta) {
      if (ir.ch >= 0 && ir.ch !== loraApp) conflict(E, now, log, ir.ch, loraApp);
      else cand = { app: loraApp, src: 'GPS' };
    } else if (ir.ch >= 0 && !ir.amb) {
      const elsewhere = fresh && T.gpsGood;
      if (!elsewhere && irStatus(E, ir.ch, now, CFG.needIr).ok) cand = { app: ir.ch, src: 'IR' };
      else if (!elsewhere) E.why = 'IR روی هد ' + NAMES[ir.ch] + ' دیده شد؛ منتظر تأیید یک‌ثانیه‌ای';
    }
    // reflections: a head that sees frames but lost to the direct path
    for (let h = 0; h < 4; h++) {
      const s = irStatus(E, h, now, 0.2);
      if (s.n >= 3 && cand && h !== cand.app && !E.refLogged) { E.refLogged = true; log('هد ' + NAMES[h] + ' فقط فریم‌های پرشدت دید (بازتاب). چون GPS و هد مستقیم مسیر ' + NAMES[cand.app] + ' را نشان می‌دهند، نادیده گرفته شد.', 'muted'); }
    }
    if (T && loraApp >= 0 && T.eta <= CFG.preEta && !E.preSent && !(E.req && E.req.app === loraApp)) {
      E.preSent = true; log('PRE: خودروی امضاشده داخل کریدور ' + NAMES[loraApp] + '، ' + Math.round(T.s) + ' متر تا خط ایست، رسیدن در ' + Math.round(T.eta) + ' ثانیه. فقط هشدار، بدون تغییر چراغ.', 'pre');
    }
    if (E.req) {
      if (cand && cand.app === E.req.app) {
        if (cand.src === 'IR+GPS' && !E.req.irLogged && (E.req.irLogged = true)) log('هد ' + NAMES[cand.app] + ' فریم‌های ۱۲ هرتز همین خودرو را گرفت؛ IR هم همان مسیر را تأیید کرد (IR+GPS).', 'on');
        E.req.support = now; E.req.src = cand.src;
      }
      if (T && fresh && T.gpsGood && T.s < CFG.passed) return release(E, now, log, 'خودرو از خط ایست عبور کرد');
      if (now - E.req.on > CFG.maxReq) return release(E, now, log, 'سقف ۱۲۰ ثانیه');
      if (now - E.req.support > CFG.hold) return release(E, now, log, 'سیگنال‌ها قطع شد (عبور یا خروج)');
      return;
    }
    if (cand && now >= E.lockUntil) {
      E.req = { app: cand.app, src: cand.src, on: now, support: now };
      log('درخواست برای مسیر ' + NAMES[cand.app] + ' از طریق ' + cand.src + '. رله‌ی همین مسیر وصل شد.', 'on');
    }
  }
  function conflict(E, now, log, irApp, gpsApp) {
    if (now - E.conflictAt < 5) return; E.conflictAt = now;
    log('تعارض جهت: IR روی هد ' + NAMES[irApp] + ' و GPS روی کریدور ' + NAMES[gpsApp] + '. درخواستی صادر نشد.', 'warn');
  }
  function release(E, now, log, why) { log('آزادسازی مسیر ' + NAMES[E.req.app] + ': ' + why + '.', 'off'); E.req = null; E.lockUntil = now + CFG.lockout; }

  // ---------------------------------------------------------------- traffic controller (lamps)
  const CT = { mode: 'normal', phase: 0, stage: 'G', ts: 0, lamps: ['G', 'R', 'G', 'R'], y: [0, 0, 0, 0], ar: 0, target: -1, preGreens: new Set() };
  function ctStep(dt) {
    if (CT.mode === 'normal') {
      CT.ts += dt;
      if (CT.stage === 'G' && CT.ts >= 12) { CT.stage = 'Y'; CT.ts = 0; }
      else if (CT.stage === 'Y' && CT.ts >= 3) { CT.stage = 'AR'; CT.ts = 0; }
      else if (CT.stage === 'AR' && CT.ts >= 2) { CT.stage = 'G'; CT.ts = 0; CT.phase ^= 1; }
      APP.forEach(a => { CT.lamps[a.i] = a.phase === CT.phase ? (CT.stage === 'G' ? 'G' : CT.stage === 'Y' ? 'Y' : 'R') : 'R'; });
      return;
    }
    const k = CT.target; let allRed = true;
    APP.forEach(a => {
      if (a.i === k && CT.lamps[k] === 'G') return;
      if (CT.lamps[a.i] === 'G') { CT.lamps[a.i] = 'Y'; CT.y[a.i] = 0; }
      if (CT.lamps[a.i] === 'Y') { CT.y[a.i] += dt; if (CT.y[a.i] >= 3) CT.lamps[a.i] = 'R'; }
      if (a.i !== k && CT.lamps[a.i] !== 'R') allRed = false;
      if (a.i === k && CT.lamps[k] === 'Y') allRed = false;
    });
    CT.ar = allRed ? CT.ar + dt : 0;
    if (CT.lamps[k] !== 'G' && CT.ar >= 2) CT.lamps[k] = 'G';
    APP.forEach(a => { if (CT.lamps[a.i] === 'G') CT.preGreens.add(a.i); });
  }
  function ctPreempt(k) { CT.mode = 'pre'; CT.target = k; CT.ar = 0; }
  function ctRelease() { const k = CT.target; CT.mode = 'normal'; CT.phase = APP[k].phase; CT.stage = 'G'; CT.ts = 6; CT.target = -1; }

  // ---------------------------------------------------------------- run state
  let S = null;
  const rnd = (() => { let s = 7; return () => (s = (s * 16807) % 2147483647) / 2147483647; })();
  function start(id, ui) {
    const sc = SCEN.find(x => x.id === id) || SCEN[0];
    let st, dir, dist;
    if (sc.from !== undefined) { const a = APP[sc.from]; st = at(a, 380); dir = a.d; dist = 380 + 12 + 150; }
    else { st = sc.start; dir = sc.dir; dist = sc.dist; }
    S = { sc, ui, t: 0, p: st.slice(), st, dir, dist, v: 14, nextIr: 0.05, nextLora: 0.3, frame: 0, counter: 1000, done: false, endT: -1,
          E: { frames: [[], [], [], []], track: null, req: null, lockUntil: 0, preSent: false, conflictAt: -99, refLogged: false, why: '' },
          headFlash: [0, 0, 0, 0], headLvl: [-1, -1, -1, -1], lastLora: -99, siren: sc.siren !== false, gps: sc.gps !== false, result: null, everReq: new Set() };
    Object.assign(CT, { mode: 'normal', phase: 1, stage: 'G', ts: 4, lamps: ['R', 'G', 'R', 'G'], y: [0, 0, 0, 0], ar: 0, target: -1, preGreens: new Set() });
    rings.forEach(r => { r.age = 99; r.m.material.opacity = 0; });
    car.rotation.z = Math.atan2(dir[1], dir[0]);
    ui.reset(sc);
    log('شروع: ' + sc.desc, 'muted');
  }
  function log(text, kind) { S.ui.log(S.t, text, kind); }

  function visible(h, lvl) {
    const a = APP[h], hv = [a.head[0] - S.p[0], a.head[1] - S.p[1]], dist = Math.hypot(hv[0], hv[1]);
    const R = (S.ui.sunny() ? 75 : 210) * [0.16, 0.5, 1][lvl];
    if (dist > R || dist < 3) return false;
    const ce = (hv[0] * S.dir[0] + hv[1] * S.dir[1]) / dist;                // emitter faces the head?
    const ch = (-hv[0] * a.u[0] - hv[1] * a.u[1]) / dist;                   // head looks at the vehicle?
    return ce >= Math.cos(EMIT * D2R) && ch >= Math.cos(FOV * D2R);
  }

  function step(dt) {
    if (!S) return;
    S.t += dt; const t = S.t;
    const travelled = Math.min(S.v * t, S.dist);
    S.p = [S.st[0] + S.dir[0] * travelled, S.st[1] + S.dir[1] * travelled];
    if (travelled >= S.dist && S.endT < 0) S.endT = t;
    // IR frames at exactly 12 per second
    while (S.siren && t >= S.nextIr) {
      const lvl = PATTERN[S.frame % 6]; S.frame++;
      beam.material.opacity = [0.10, 0.16, 0.24][lvl]; beam.userData.until = t + 0.035;
      for (let h = 0; h < 4; h++) if (visible(h, lvl) && rnd() < 0.92) {
        S.E.frames[h].push({ t: S.nextIr, lvl }); S.headFlash[h] = t + 0.09; S.headLvl[h] = lvl;
        setLine(rxLines[h], [S.p[0], S.p[1], 2.5], [APP[h].head[0], APP[h].head[1], 5.6], 0.95, t + 0.09);
      }
      if (S.sc.reflect !== undefined) {
        const a = APP[S.sc.from], e = S.p[0] - a.stop[0], n = S.p[1] - a.stop[1], s = e * a.u[0] + n * a.u[1];
        if (lvl === 2 && s > 20 && s < 130 && rnd() < 0.6) {
          const h = S.sc.reflect; S.E.frames[h].push({ t: S.nextIr, lvl }); S.headFlash[h] = t + 0.09; S.headLvl[h] = lvl;
          const g = refLine.geometry.attributes.position;
          g.setXYZ(0, S.p[0], S.p[1], 2.5); g.setXYZ(1, -22, 30, 8); g.setXYZ(2, APP[h].head[0], APP[h].head[1], 5.6); g.needsUpdate = true;
          refLine.material.opacity = 0.9; refLine.userData.until = t + 0.09;
        }
      }
      S.E.frames.forEach(F => { while (F.length && t - F[0].t > 2.6) F.shift(); });
      S.nextIr += PERIOD;
    }
    // LoRa beacon, once per second, signed
    if (S.siren && t >= S.nextLora) {
      S.nextLora += 1; S.counter++;
      const ring = rings.find(r => r.age > 2) || rings[0];
      ring.age = 0; ring.m.position.set(S.p[0], S.p[1], 1);
      setLine(loraLine, [S.p[0], S.p[1], 2.6], [CAB[0], CAB[1], 3.7], 0.9, t + 0.35);
      ant.material.emissive.setHex(0x3b82f6); ant.userData.until = t + 0.35;
      const noise = () => (rnd() - 0.5) * 6;
      const hdg = (Math.atan2(S.dir[0], S.dir[1]) / D2R + 360 + (rnd() - 0.5) * 10) % 360;
      const E = S.E; if (!E.track) E.track = { app: -1, s: 0, prevS: 1e9, nMatch: 0, eta: -1 };
      E.track.t = t; E.track.counter = S.counter;
      locate(E.track, [S.p[0] + noise(), S.p[1] + noise()], hdg, S.v, S.gps);
      if (S.gps && E.track.app < 0 && !S.outLogged && !E.req) { S.outLogged = true; log('بسته‌ی LoRa امضاشده رسید، اما موقعیت و جهت خودرو با کریدور هیچ مسیری نمی‌خواند. درخواستی صادر نمی‌شود.', 'muted'); }
      S.lastLora = t;
    }
    engineTick(S.E, t, log);
    if (S.E.req && CT.mode === 'normal') { ctPreempt(S.E.req.app); S.everReq.add(S.E.req.app); }
    if (!S.E.req && CT.mode === 'pre') ctRelease();
    ctStep(dt);
    // finish a few seconds after the run so the release is visible
    if (S.endT >= 0 && t > S.endT + 9 && !S.done) finish();
  }
  function finish() {
    S.done = true;
    const exp = S.sc.expect, greens = [...CT.preGreens];
    let ok, text;
    if (exp < 0) { ok = S.everReq.size === 0; text = ok ? 'هیچ درخواستی صادر نشد و چراغ‌ها در چرخه‌ی عادی ماندند.' : 'درخواست غیرمنتظره برای ' + [...S.everReq].map(i => NAMES[i]).join('، '); }
    else {
      const onlyRight = S.everReq.size === 1 && S.everReq.has(exp) && greens.every(g => g === exp);
      ok = onlyRight && greens.includes(exp);
      text = ok ? 'در حالت اولویت فقط چراغ مسیر ' + NAMES[exp] + ' سبز شد؛ سه مسیر دیگر قرمز ماندند.' : 'نتیجه با انتظار نخواند: ' + greens.map(i => NAMES[i]).join('، ');
    }
    S.result = { ok, text };
    S.ui.result(ok, text);
  }
  function setLine(l, a, b, op, until) {
    const g = l.geometry.attributes.position; g.setXYZ(0, ...a); g.setXYZ(1, ...b); g.needsUpdate = true;
    l.material.opacity = op; l.userData.until = until;
  }

  // ---------------------------------------------------------------- per-frame visuals
  function render() {
    if (!S) return;
    const t = S.t;
    car.position.set(S.p[0], S.p[1], 0);
    const blink = Math.floor(t * 4) % 2 === 0;
    barR.material.color.setHex(S.siren && blink ? 0xff2020 : 0x551010);
    barB.material.color.setHex(S.siren && !blink ? 0x2060ff : 0x101a55);
    if (!(beam.userData.until > t)) beam.material.opacity = 0;
    [...rxLines, loraLine, refLine].forEach(l => { if (!(l.userData.until > t)) l.material.opacity = 0; });
    if (!(ant.userData.until > t)) ant.material.emissive.setHex(0x000000);
    rings.forEach(r => { if (r.age < 2) { const R = 10 + r.age * 260; r.m.scale.set(R, R, 1); r.m.material.opacity = 0.45 * (1 - r.age / 2); } else r.m.material.opacity = 0; });
    APP.forEach(a => {
      const L = CT.lamps[a.i];
      stopBars[a.i].material.color.setHex(lampColor[L]);
      ['R', 'Y', 'G'].forEach(c => lamps[a.i][c].material.color.setHex(L === c ? lampColor[c] : 0x2a2e33));
      const on = S.headFlash[a.i] > t;
      heads[a.i].material.emissive.setHex(on ? 0xf0ab1f : 0x000000);
      wedges[a.i].material.opacity = S.ui.showFov() ? (on ? 0.28 : 0.10) : 0;
      corridors[a.i].material.opacity = S.ui.showCorr() ? (S.E.track && S.E.track.app === a.i && S.gps ? 0.20 : 0.07) : 0;
    });
  }
  function advance(dtReal, speed) {
    if (!S || S.paused || S.done) { render(); return; }
    let left = Math.min(dtReal, 0.1) * speed;
    while (left > 0) { const d = Math.min(left, 0.02); step(d); for (const r of rings) r.age += d; left -= d; }
    render();
  }

  // ---------------------------------------------------------------- read-outs for the panel
  function status() {
    if (!S) return null;
    const t = S.t, E = S.E, T = E.track;
    const rows = APP.map(a => {
      const st = irStatus(E, a.i, t, CFG.need);
      let gps = '—';
      if (T && t - T.t <= 3) gps = !T.gpsGood ? 'بدون فیکس' : T.app === a.i ? 'داخل کریدور · ' + Math.round(T.s) + ' m' : '—';
      return { name: a.name, irN: st.n, irLvl: st.best, irOk: st.ok, gps, lamp: CT.lamps[a.i] };
    });
    const loraAge = t - S.lastLora;
    return {
      t, rows, siren: S.siren,
      lora: S.siren ? (loraAge < 5 ? 'آخرین بسته ' + loraAge.toFixed(1) + ' ثانیه پیش · امضای HMAC معتبر · شمارنده ' + S.counter : 'منتظر بسته') : 'خاموش (آژیر خاموش)',
      gps: !S.siren ? '—' : !S.gps ? 'بدون فیکس (دره‌ی شهری)' : T && T.app >= 0 ? 'کریدور ' + NAMES[T.app] + ' · ' + Math.round(T.s) + ' m تا خط ایست · رسیدن در ' + Math.max(0, Math.round(T.eta)) + ' s' : 'بیرون از همه‌ی کریدورها',
      verdict: E.req ? { on: true, text: 'درخواست فعال برای مسیر ' + NAMES[E.req.app] + ' (' + E.req.src + ')' } : { on: false, text: E.why || 'بدون درخواست — چرخه‌ی عادی چراغ‌ها' },
      ctrl: CT.mode === 'pre' ? 'حالت اولویت: پاک‌سازی مسیرهای دیگر (زرد ۳ ثانیه، تمام‌قرمز ۲ ثانیه)، سپس سبز فقط برای ' + NAMES[CT.target] : 'چرخه‌ی عادی: شمال-جنوب و شرق-غرب به نوبت',
      done: S.done, result: S.result,
    };
  }
  function labels() {
    if (!S) return [];
    const L = [{ text: 'آمبولانس', p: [S.p[0], S.p[1], 5], cls: 'veh' }, { text: 'کنترلر کابینت + آنتن LoRa', p: [CAB[0], CAB[1], 5], cls: 'cab' }, { text: 'خیابان موازی', p: [80, 150, 1], cls: 'muted' }];
    APP.forEach(a => L.push({ text: 'هد ' + a.name, p: [a.head[0], a.head[1], 7.5], cls: S.headFlash[a.i] > S.t ? 'hot' : '' }));
    return L;
  }
  return {
    group: G, scenarios: SCEN, start, advance, status, labels,
    pause(on) { if (S) S.paused = on; }, paused() { return S ? !!S.paused : true; },
    vehicle() { return S ? S.p : [0, 0]; },
  };
};
