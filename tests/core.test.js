const test = require('node:test'), assert = require('node:assert');
const T = require('../src/traffic-core.js');

function conflicts(w) {
  let v = 0;
  [w.base, w.smart].forEach(s => s.net.nodes.forEach(n => {
    const occ = n.connList.filter(c => c.vehicles.length);
    for (const a of occ) for (const b of occ) if (a !== b && a.conflicts.includes(b)) v++;
  }));
  return v;
}

for (const scn of Object.keys(T.SCENARIOS)) for (const mode of ['protected', 'permissive']) {
  test(`safety kernel: no conflicting movements at once (${scn}, ${mode})`, () => {
    const w = new T.World(scn, { leftMode: mode }, 7); w.setRate(900);
    let bad = 0;
    for (let i = 0; i < 6000; i++) {
      if (i === 1500) w.addEV('amb'); if (i === 2500) w.addEV('fire'); if (i === 3000) w.addEV('police');
      w.step(0.1); if (i % 5 === 0) bad += conflicts(w);
    }
    assert.strictEqual(bad, 0);
  });
}

test('adaptive control beats fixed-time by >=40% delay in heavy traffic (cross, protected left)', () => {
  let g = 0; const K = 3;
  for (let k = 0; k < K; k++) {
    const w = new T.World('cross', { leftMode: 'protected' }, 101 + k); w.setRate(950);
    for (let i = 0; i < 3000; i++) w.step(0.1); w.resetStats();
    for (let i = 0; i < 12000; i++) w.step(0.1);
    g += 1 - w.smart.metrics().avgStopAll / w.base.metrics().avgStopAll;
  }
  assert.ok(g / K >= 0.40, 'gain ' + (g / K));
});

test('ambulance crosses with (almost) no stop under the smart controller', () => {
  const w = new T.World('cross', { leftMode: 'protected' }, 5); w.setRate(500);
  for (let i = 0; i < 3000; i++) w.step(0.1); w.resetStats(); w.addEV('amb');
  for (let i = 0; i < 2400; i++) w.step(0.1);
  const e = w.smart.stats.ev[0]; assert.ok(e, 'ambulance finished'); assert.ok(e.stopped < 5, 'stopped ' + e.stopped);
  assert.strictEqual(w.smart.stats.risky, 0);
});

test('watchdog falls back to the fixed plan when sensors fail', () => {
  const w = new T.World('cross', { leftMode: 'protected', fault: true }, 3); w.setRate(500);
  for (let i = 0; i < 600; i++) w.step(0.1);
  assert.strictEqual(w.smart.net.nodes[0].fallback, true);
});

test('deterministic: same seed gives same result', () => {
  const run = () => { const w = new T.World('cross', {}, 42); w.setRate(600); for (let i = 0; i < 3000; i++) w.step(0.1); return w.smart.metrics().done; };
  assert.strictEqual(run(), run());
});
