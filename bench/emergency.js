#!/usr/bin/env node
/* One ambulance enters at t=300 s from a random approach (same one for both controllers). Time from entry to exit and time spent stopped. */
const fs = require('fs'), path = require('path');
const T = require('../src/traffic-core.js');
const RATES = [300, 500, 750, 950, 1100], K = 12, out = [];
for (const rate of RATES) {
  const a = { time: [], stop: [], risky: 0 }, b = { time: [], stop: [] };
  for (let k = 0; k < K; k++) {
    const w = new T.World('cross', { leftMode: 'protected' }, 201 + k); w.setRate(rate);
    for (let i = 0; i < 3000; i++) w.step(0.1);
    w.resetStats(); w.addEV('amb');
    for (let i = 0; i < 2400; i++) w.step(0.1);
    const ea = w.base.stats.ev[0], eb = w.smart.stats.ev[0];
    if (ea && eb) { a.time.push(ea.time); a.stop.push(ea.stopped); if (ea.risky) a.risky++; b.time.push(eb.time); b.stop.push(eb.stopped); }
  }
  const m = x => x.reduce((s, v) => s + v, 0) / x.length;
  const row = { rate, n: a.time.length, fixed: { time: m(a.time), stop: m(a.stop), risky: a.risky }, smart: { time: m(b.time), stop: m(b.stop) } };
  out.push(row);
  console.log(`load ${rate}: ambulance ${row.fixed.time.toFixed(1)} s (stopped ${row.fixed.stop.toFixed(1)}, red-runs ${a.risky}/${row.n}) -> ${row.smart.time.toFixed(1)} s (stopped ${row.smart.stop.toFixed(1)})`);
}
fs.writeFileSync(path.join(__dirname, '..', 'results', 'emergency.json'), JSON.stringify(out, null, 1));
