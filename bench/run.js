#!/usr/bin/env node
/* Benchmark matrix: scenario x load x left-turn mode.  Usage: node bench/run.js <scenario> <protected|permissive> [seeds]
   Writes results/<scenario>-<mode>.json.  Same random demand drives both controllers (paired comparison). */
const fs = require('fs'), path = require('path');
const T = require('../src/traffic-core.js');
const scn = process.argv[2] || 'cross', mode = process.argv[3] || 'protected', K = +(process.argv[4] || 5);
const RATES = { cross: [300, 500, 750, 950, 1100, 1300], tee: [300, 500, 750, 950], corridor: [300, 600, 900], grid: [300, 600, 900] }[scn];
const WARM = 300, MEAS = 1500, SEED0 = 101;      // seeds 101.. were never used while tuning (tuning used 1-2, 11-14)
const out = [];
for (const rate of RATES) {
  const runs = [];
  for (let k = 0; k < K; k++) {
    const w = new T.World(scn, { leftMode: mode }, SEED0 + k); w.setRate(rate);
    for (let i = 0; i < WARM * 10; i++) w.step(0.1);
    w.resetStats();
    const q = { a: 0, b: 0, n: 0 }, mw = { a: 0, b: 0 }; let viol = 0;
    const d0 = [w.base.stats.n, w.smart.stats.n];
    for (let i = 0; i < MEAS * 10; i++) {
      w.step(0.1);
      if (i % 100 === 0) {
        const a = w.base.metrics(), b = w.smart.metrics(); q.a += a.queue; q.b += b.queue; q.n++;
        mw.a = Math.max(mw.a, a.maxWait); mw.b = Math.max(mw.b, b.maxWait);
      }
      if (i % 5 === 0) [w.base, w.smart].forEach(s => s.net.nodes.forEach(n => {
        const o = n.connList.filter(c => c.vehicles.length);
        for (const x of o) for (const y of o) if (x !== y && x.conflicts.includes(y)) viol++; }));
    }
    const a = w.base.metrics(), b = w.smart.metrics();
    runs.push({ seed: SEED0 + k, fixed: { delayAll: a.avgStopAll, delayDone: a.avgStop, queue: q.a / q.n, maxWait: mw.a, thr: (w.base.stats.n - d0[0]) * 3600 / MEAS },
      smart: { delayAll: b.avgStopAll, delayDone: b.avgStop, queue: q.b / q.n, maxWait: mw.b, thr: (w.smart.stats.n - d0[1]) * 3600 / MEAS }, conflicts: viol });
  }
  const mean = (f) => runs.reduce((s, r) => s + f(r), 0) / runs.length;
  const sd = (f) => { const m = mean(f); return Math.sqrt(runs.reduce((s, r) => s + (f(r) - m) ** 2, 0) / runs.length); };
  const gain = (r) => 1 - r.smart.delayAll / r.fixed.delayAll, gainDone = (r) => 1 - r.smart.delayDone / r.fixed.delayDone;
  const row = { scenario: scn, mode, rate, seeds: K,
    fixed: { delayAll: mean(r => r.fixed.delayAll), delayDone: mean(r => r.fixed.delayDone), queue: mean(r => r.fixed.queue), maxWait: mean(r => r.fixed.maxWait), thr: mean(r => r.fixed.thr) },
    smart: { delayAll: mean(r => r.smart.delayAll), delayDone: mean(r => r.smart.delayDone), queue: mean(r => r.smart.queue), maxWait: mean(r => r.smart.maxWait), thr: mean(r => r.smart.thr) },
    gainAll: mean(gain), gainAllSd: sd(gain), gainDone: mean(gainDone), conflicts: runs.reduce((s, r) => s + r.conflicts, 0) };
  out.push(row);
  console.log(`${scn}/${mode} rate ${rate}: delay ${row.fixed.delayAll.toFixed(1)} -> ${row.smart.delayAll.toFixed(1)} s  gain ${(row.gainAll * 100).toFixed(0)}% ±${(row.gainAllSd * 100).toFixed(0)}  conflicts ${row.conflicts}`);
}
fs.mkdirSync(path.join(__dirname, '..', 'results'), { recursive: true });
fs.writeFileSync(path.join(__dirname, '..', 'results', `${scn}-${mode}.json`), JSON.stringify(out, null, 1));
