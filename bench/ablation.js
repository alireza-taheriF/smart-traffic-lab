#!/usr/bin/env node
/* Ablation: which idea contributes how much?  Cross, protected left, seeds 101-105. */
const fs = require('fs'), path = require('path');
const T = require('../src/traffic-core.js');
const FULL = Object.assign({}, T.TUNE);
const VARIANTS = [
  ['full', {}],
  ['without dynamic all-red', { dynAR: 0 }],
  ['without left-phase skipping (leftThr=0.8)', { leftThr: 0.8 }],
  ['long minimum green (25 s) / lazy switching', { minT: 25, ratio: 3, off: 8 }],
  ['no queue-aware green (gap-out only, ratio=99)', { ratio: 99, off: 99 }],
  ['without anti-starvation weight (waitDiv=1e9)', { waitDiv: 1e9 }],
  ['coarse gap-out (gap=6 s)', { gap: 6 }]
];
const RATES = [750, 950, 1100], K = 5, res = [];
for (const [name, cfg] of VARIANTS) {
  const row = { name, gains: [] };
  for (const rate of RATES) {
    let g = 0;
    for (let k = 0; k < K; k++) {
      Object.assign(T.TUNE, FULL, cfg);
      const w = new T.World('cross', { leftMode: 'protected' }, 101 + k); w.setRate(rate);
      for (let i = 0; i < 3000; i++) w.step(0.1); w.resetStats();
      for (let i = 0; i < 15000; i++) w.step(0.1);
      g += 1 - w.smart.metrics().avgStopAll / w.base.metrics().avgStopAll;
    }
    row.gains.push(g / K);
  }
  Object.assign(T.TUNE, FULL);
  res.push(row);
  console.log(name.padEnd(52), row.gains.map(x => (x * 100).toFixed(0) + '%').join('  '));
}
fs.writeFileSync(path.join(__dirname, '..', 'results', 'ablation.json'), JSON.stringify({ rates: RATES, rows: res }, null, 1));
